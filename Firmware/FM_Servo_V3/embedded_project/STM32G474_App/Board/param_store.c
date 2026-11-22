/*
 * param_store.c —— 辨识/标定参数的 Flash 持久化
 *
 * 编程单位：STM32G4 只支持 64-bit（doubleword）编程，所以 ParamBlob 必须
 * 按 8 字节对齐、按 8 字节写。结构体设计成 48 B = 6 个 doubleword。
 *
 * 为什么不放在"选项字节"或"备份寄存器"里：
 *   - 选项字节是给芯片配置用的，改写有锁死风险；
 *   - G4 的 BKPxR 一共几十字节，放不下 48 B，而且掉电靠 VBAT，
 *     本板没有纽扣电池。
 *   Flash 最后一页最合适：容量足够、擦写可控、不掉电就一直在。
 */
#include "param_store.h"
#include "bsp_servo.h"

#include <string.h>

/* 编译期护栏：结构体必须正好 7 个 doubleword（56 B），否则写出去的布局与读回不一致。
 * v2 从 48 B 扩到 56 B（加 fric_a / fric_visc），PS_VERSION 同步升到 2 ——
 * 旧参数会因 version 不符被判"没存过"，这是有意的：字段变了就不能半套应用。 */
/* v3：56 -> 64 B（加 psi_wb + 补齐）。PS_VERSION 同步升到 3，
 * 旧参数会因 version 不符被判"没存过"，这是有意的。 */
typedef char ps_size_check[(sizeof(ParamBlob) == 64u) ? 1 : -1];
typedef char ps_align_check[((_Alignof(ParamBlob) <= 8u) &&
                             (sizeof(ParamBlob) % 8u == 0u)) ? 1 : -1];

#define PS_PAGE_SIZE    (0x1000u)       /* 4 KB：512KB / 128 页 */
#define PS_DW           (8u)            /* doubleword = 8 B，G4 的最小编程单位 */
#define PS_BANK2_BASE   (0x08040000u)

static volatile uint8_t s_ps_busy;
static ParamStoreDiag    s_diag;

/* 编程源缓冲必须是 8 字节对齐：G4 的 HAL_FLASH_Program(DOUBLEWORD) 把 Data
 * 当作指向 8 字节的指针，非对齐地址读出来就是垃圾。
 * 用静态缓冲而不是局部变量 —— 局部变量即使结构体带 aligned(8)，
 * 实测仍会落在 4 字节边界上（编译器只按成员最大对齐 4 来排栈）。 */
static ParamBlob s_wbuf __attribute__((aligned(8)));

uint8_t ParamStore_IsBusy(void) { return s_ps_busy; }
const ParamStoreDiag *ParamStore_GetDiag(void) { return &s_diag; }

/* CRC-32 (IEEE 802.3, 反射多项式 0xEDB88320)，逐位实现：
 * 只有 48 字节，没必要为它拉一张 1 KB 的表进 Flash。 */
static uint32_t crc32_calc(const uint8_t *p, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFu;

    for (uint32_t i = 0; i < n; i++) {
        c ^= p[i];
        for (uint8_t k = 0; k < 8u; k++) {
            uint32_t mask = (uint32_t)(-(int32_t)(c & 1u));
            c = (c >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~c;
}

static uint32_t blob_crc(const ParamBlob *b)
{
    /* 只覆盖 crc32 字段之前的字节：crc32 自身与 pad 不参与 */
    return crc32_calc((const uint8_t *)b, (uint32_t)offsetof(ParamBlob, crc32));
}

/* 上次 Load 失败的原因（2026-11-21）。为什么要单独记：
 * 上电时只打一句 "ps:empty"，cal/z 就变成 0，现场只能靠猜是"从来没存过"
 * 还是"存过但被擦/被改"。四道检查各自对应完全不同的处理：
 *   magic   → 页被擦过（烧录/擦除/扇区被误擦）
 *   version → 固件布局升级，旧 blob 主动作废（这是设计行为）
 *   crc     → 页里内容坏了（写一半掉电 / 读回错）
 *   flags   → 存的是半套（例如只有 C 没有 Z），按设计拒绝应用
 * 只读暴露在 30082 整数槽 13（ps_diag = why | ver<<8）。 */
static uint8_t  s_load_why;
static uint16_t s_load_ver;

uint8_t  ParamStore_GetLoadWhy(void)      { return s_load_why; }
uint16_t ParamStore_GetLoadVer(void)      { return s_load_ver; }

uint8_t ParamStore_Load(ParamBlob *out)
{
    ParamBlob tmp;

    s_load_why = 0u;
    s_load_ver = 0u;

    memcpy(&tmp, (const void *)PS_FLASH_ADDR, sizeof(tmp));
    s_load_ver = tmp.version;

    if (tmp.magic != PS_MAGIC)          { s_load_why = 1u; return 0U; }
    if (tmp.version != PS_VERSION)      { s_load_why = 2u; return 0U; }
    if (tmp.crc32 != blob_crc(&tmp))    { s_load_why = 3u; return 0U; }
    if ((tmp.flags & (PS_FLAG_BIAS | PS_FLAG_ZERO)) == 0U) { s_load_why = 4u; return 0U; }

    if (out != 0) { *out = tmp; }
    return 1U;
}

/*
 * 判断目标页属于哪个 bank。
 * G474CE 支持 DBANK 双 bank 选项：DBANK=1 时前 256 KB 是 bank1、后 256 KB 是 bank2，
 * 页号在各自 bank 内从 0 重新计数；DBANK=0 时整片单 bank，页号 0..127。
 * 搞错 bank 的表现是 HAL_FLASHEx_Erase 返回错误（或更糟：擦掉别的页），
 * 所以每次都从 OPTR 实时读，不靠编译期假设。
 */
static uint8_t ps_page_of_addr(uint32_t addr, uint32_t *bank,
                               uint32_t *page, uint32_t *psize)
{
#if defined(FLASH_OPTR_DBANK)
    s_diag.dbank = ((FLASH->OPTR & FLASH_OPTR_DBANK) != 0u) ? 1u : 0u;
    if (s_diag.dbank != 0u) {
        if (addr < PS_BANK2_BASE) { return 0U; }   /* 本布局只放 bank2 */
        *bank  = FLASH_BANK_2;
        /* 双 bank：每 bank 256 KB / 128 页 = 2 KB/页。
         * 这里踩过坑：一开始按 4 KB 算，页号算成 63，实际擦的是
         * 0x0805F800 那一页，目标页根本没擦 —— 编程返回 OK 但回读 CRC 不对。 */
        *psize = 0x800u;
        *page  = (addr - PS_BANK2_BASE) / *psize;
        return 1U;
    }
#else
    s_diag.dbank = 0u;
#endif
    *bank  = FLASH_BANK_1;
    *psize = PS_PAGE_SIZE;                                  /* 单 bank：4 KB/页 */
    *page  = (addr - 0x08000000u) / *psize;
    return 1U;
}

/* 现有调用都针对 ParamBlob 那一页，保持原签名 */
static uint8_t ps_page_of(uint32_t *bank, uint32_t *page, uint32_t *psize)
{
    return ps_page_of_addr(PS_FLASH_ADDR, bank, page, psize);
}

uint8_t ParamStore_Save(ParamBlob *in)
{
    ParamBlob cur;
    FLASH_EraseInitTypeDef er;
    uint32_t page_err = 0U;
    uint32_t bank = 0U, page = 0U, psize = 0U;
    uint8_t  ok = 1U;

    if (in == 0) { return 0U; }

    /* MOE 开着时绝不擦写：单 bank 下擦除会让 CPU 停顿 20~40 ms，
     * 电流环失去节拍而占空比停在最后一次写入值 —— 等于桥臂输出冻结。 */
    if (BspServo_IsPwmEnabled() != 0U) { return 0U; }

    s_wbuf = *in;
    s_wbuf.magic   = PS_MAGIC;
    s_wbuf.version = PS_VERSION;
    s_wbuf.crc32 = blob_crc(&s_wbuf);

    /* 与已存内容完全相同就不写：调试时反复 C/Z 不会把 10k 次擦写寿命耗光 */
    if ((ParamStore_Load(&cur) != 0U) &&
        (memcmp(&cur, &s_wbuf, sizeof(s_wbuf)) == 0)) {
        return 1U;
    }

    if (ps_page_of(&bank, &page, &psize) == 0U) { return 0U; }
    s_diag.bank      = bank;
    s_diag.page      = page;
    s_diag.page_size = psize;
    s_diag.prog_ret  = 0U;
    s_diag.readback  = 0U;

    s_ps_busy = 1U;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    memset(&er, 0, sizeof(er));
    er.TypeErase = FLASH_TYPEERASE_PAGES;
    er.Banks     = bank;
    er.Page      = page;
    er.NbPages   = 1U;

    s_diag.erase_ret = (uint32_t)HAL_FLASHEx_Erase(&er, &page_err);
    if (s_diag.erase_ret != (uint32_t)HAL_OK) { ok = 0U; }

    s_diag.align = (uint32_t)(uintptr_t)&s_wbuf & 0x7u;
    if (ok != 0U) {
        const uint32_t *src = (const uint32_t *)&s_wbuf;
        if (s_diag.align != 0u) {
            ok = 0U;                    /* 理论到不了这里；真到了就在回显里能看到 */
        } else {
            for (uint32_t i = 0U; i < (sizeof(ParamBlob) / PS_DW); i++) {
                /*
                 * ⚠ 这里必须按**值**传 8 字节。
                 * STM32G4 的签名是 HAL_FLASH_Program(TypeProgram, Address, uint64_t Data)，
                 * Data 是数据本身；而 H7/H5 那几代的签名是 uint32_t Data 且内部当指针用。
                 * 按 H7 的习惯传 &buf 会把**指针值**写进 Flash —— 编程返回 HAL_OK、
                 * 回读 CRC 不对，现象与"没擦干净"完全一样，极难定位（实测踩过）。
                 * 用 memcpy 组装，顺便免掉对源缓冲的对齐要求。
                 */
                uint64_t dw = 0U;
                memcpy(&dw, ((const uint8_t *)&s_wbuf) + (i * PS_DW), PS_DW);

                HAL_StatusTypeDef ps = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                                         PS_FLASH_ADDR + (i * PS_DW),
                                                         dw);
                if (ps != HAL_OK) {
                    s_diag.prog_ret = (uint32_t)ps;
                    ok = 0U;
                    break;
                }
            }
        }
    }

    HAL_FLASH_Lock();
    s_ps_busy = 0U;

    if (ok == 0U) { return 0U; }

    /* 回读校验：不信任"编程返回 OK"，只信任读回来的内容 */
    s_diag.readback = (uint32_t)ParamStore_Load(0);
    return (uint8_t)s_diag.readback;
}

uint8_t ParamStore_Erase(void)
{
    FLASH_EraseInitTypeDef er;
    uint32_t page_err = 0U;
    uint32_t bank = 0U, page = 0U, psize = 0U;

    if (BspServo_IsPwmEnabled() != 0U) { return 0U; }
    if (ps_page_of(&bank, &page, &psize) == 0U) { return 0U; }
    s_diag.bank      = bank;
    s_diag.page      = page;
    s_diag.page_size = psize;
    s_diag.prog_ret  = 0U;
    s_diag.readback  = 0U;

    s_ps_busy = 1U;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    memset(&er, 0, sizeof(er));
    er.TypeErase = FLASH_TYPEERASE_PAGES;
    er.Banks     = bank;
    er.Page      = page;
    er.NbPages   = 1U;

    uint8_t ok = (HAL_FLASHEx_Erase(&er, &page_err) == HAL_OK) ? 1U : 0U;

    HAL_FLASH_Lock();
    s_ps_busy = 0U;
    return ok;
}

/* =========================================================================
 * 齿槽表（单独一页）
 * =========================================================================
 * 布局：24 B 头 + 128×int16 = 280 B = 35 个 doubleword。
 * 尾部不补齐也没问题（280 已经是 8 的倍数）。
 *
 * zero_ref 存的是**测量当时的编码器零点**：齿槽表按 θ_multi 的 0~2π 分箱，
 * 而 θ_multi 的零点就是编码器零点。重新 Z 后零点变，表的相位就整体偏了，
 * 错相位的前馈比没有前馈更糟（会把扰动放大），所以加载时必须比对。
 */
#define COG_FLASH_ADDR   (0x0807E000u)
#define COG_MAGIC        (0x43323758u)     /* 'X72C' */
#define COG_VERSION      (3u)              /* v3: 表 128→512 箱（1056 B）；v2: 加 en、288 B、
                                            *     CRC 覆盖整表
                                            * v1: 280 B，crc32 排在 cog[] 之前，
                                            *     实际只保护了 20 B 头，表数据裸奔。
                                            * 版本不符一律当"没存过"，所以升版即作废旧表。 */

/* 布局要点（288 B = 36 个 doubleword，必须整除，否则末尾字节写不进去）：
 *
 *   magic(4) version(2) n_bins(2) en(2) rsv[3](6) | zero_ref(4) pp(4) phase(4)
 *   | cog[128](256) | crc32(4)
 *
 * rsv[3] 是**补齐**而不是预留：cog[] 要落在 8 的倍数上（28→32 之前是 28，
 * 其实 28 也满足 doubleword 的 memcpy 读取，但补齐后头部 16 B 对齐更整齐，
 * 且总长凑到 288 才能被 PS_DW=8 整除）。
 *
 * crc32 放在最后：v1 把它放在 cog[] 前面，于是 CRC 只覆盖结构体头，
 * 齿槽表本身一位都没保护 —— 表里错一个点不会报警，只会让补偿反相。
 */
typedef struct __attribute__((aligned(8))) {
    uint32_t magic;
    uint16_t version;
    uint16_t n_bins;
    uint16_t en;          /* 1 = 用户已开启（加载时还要过零点校验才算数） */
    uint16_t rsv[3];      /* 补齐到 16 B */
    float    zero_ref;    /* 16 */
    float    pp;          /* 20 */
    float    phase;       /* 24 */
    int16_t  cog[COG_BINS_MAX];   /* 28..1051 */
    uint32_t crc32;       /* 1052，覆盖之前全部字节（含整张表） */
} CogBlob;                                  /* 1056 B = 132 个 doubleword */

typedef char cog_size_check[((sizeof(CogBlob) == 1056u) &&
                             ((sizeof(CogBlob) % 8u) == 0u)) ? 1 : -1];

/* 静态对齐缓冲：理由与 ParamStore_Save 相同（栈上结构体只有 4 B 对齐） */
static CogBlob s_cbuf __attribute__((aligned(8)));

static uint32_t cog_crc(const CogBlob *b)
{
    /* 覆盖 crc32 之前的全部字节：头部 + **整张 128 点表**。
     * v1 只盖到 20 B 头，表数据坏了查不出来，所以这次把 crc32 挪到结构体末尾。 */
    return crc32_calc((const uint8_t *)b, (uint32_t)offsetof(CogBlob, crc32));
}

uint8_t ParamStore_LoadCog(int16_t *tab, uint16_t *n_bins,
                           float *zero_ref, float *phase, float *pp,
                           uint8_t *en)
{
    const CogBlob *src = (const CogBlob *)COG_FLASH_ADDR;

    if ((src->magic != COG_MAGIC) || (src->version != COG_VERSION)) { return 0U; }
    if ((src->n_bins == 0U) || (src->n_bins > COG_BINS_MAX))        { return 0U; }

    memcpy(&s_cbuf, src, sizeof(s_cbuf));       /* 先落到对齐缓冲再算 CRC */

    if (cog_crc(&s_cbuf) != s_cbuf.crc32) { return 0U; }

    if (tab)      { memcpy(tab, s_cbuf.cog, (size_t)s_cbuf.n_bins * sizeof(int16_t)); }
    /* n_bins 是 uint16_t，**不能再转 uint8_t**：512 箱会让高位截掉变 0，
     * 而调用方 CogFf_LoadFromFlash 拿 0 去 cog_install 会直接 return，
     * 现象是"表存进去了、CRC 也过、但每次上电 n_bins=0、齿槽补偿恒为 0"。
     * 这是 128→512 箱迁移时漏掉的一处（128 时恰好放得进一个字节）。 */
    if (n_bins)   { *n_bins = s_cbuf.n_bins; }
    if (zero_ref) { *zero_ref = s_cbuf.zero_ref; }
    if (phase)    { *phase = s_cbuf.phase; }
    if (pp)       { *pp = s_cbuf.pp; }
    if (en)       { *en = (s_cbuf.en != 0U) ? 1U : 0U; }
    return 1U;
}

uint8_t ParamStore_HasCog(void)
{
    return ParamStore_LoadCog(NULL, NULL, NULL, NULL, NULL, NULL);
}

uint8_t ParamStore_SaveCog(const int16_t *tab, uint16_t n_bins,
                           float zero_ref, float phase, float pp, uint8_t en)
{
    FLASH_EraseInitTypeDef er;
    uint32_t page_err = 0U;
    uint32_t bank = 0U, page = 0U, psize = 0U;
    uint8_t  ok = 1U;
    uint32_t i;

    if (BspServo_IsPwmEnabled() != 0U)  { return 0U; }   /* 擦除会让桥臂冻结 */
    if (tab == NULL)                    { return 0U; }
    if ((n_bins == 0U) || (n_bins > COG_BINS_MAX)) { return 0U; }

    memset(&s_cbuf, 0xFF, sizeof(s_cbuf));
    s_cbuf.magic    = COG_MAGIC;
    s_cbuf.version  = COG_VERSION;
    s_cbuf.n_bins   = n_bins;
    s_cbuf.en       = (en != 0U) ? 1U : 0U;
    s_cbuf.zero_ref = zero_ref;
    s_cbuf.phase    = phase;
    s_cbuf.pp       = pp;
    memcpy(s_cbuf.cog, tab, (size_t)n_bins * sizeof(int16_t));
    s_cbuf.crc32    = cog_crc(&s_cbuf);

    /* 内容没变就不写：齿槽表要反复标定时省寿命。
     * 开关也算内容 —— 只切 `;` 不改表、但要让 en 落盘时，必须真写一次，
     * 否则 "开了却存不进去" 会变成很难查的哑故障。
     *
     * ⚠⚠ 2026-11-18：这里原来调 ParamStore_LoadCog() 做比较，而 LoadCog 内部是
     *     memcpy(&s_cbuf, src, sizeof(s_cbuf));
     * —— 它把**刚刚填好的待写数据 s_cbuf 直接覆盖成了 Flash 里的旧数据**，
     * 于是 memcmp 永远相等、永远 return 1 跳过写入。
     * 外表现象：`A` 回显 "PS saved ... +cog"、ParamStore_HasCog() 的 CRC 校验也通过
     * （因为 Flash 里确实是合法旧表），但**一个字节都没写**，复位后回到旧表。
     * 这个别名 bug 让齿槽表持久化从实现起就没真正工作过。
     *
     * 修法：直接读 Flash 指针比较，不再经过 s_cbuf。CogBlob 在 0x0807E000 上是
     * 8 字节对齐的，可以直接按结构体访问；CRC 先验，再逐字段比。
     */
    {
        const CogBlob *fw = (const CogBlob *)COG_FLASH_ADDR;

        if ((fw->magic   == COG_MAGIC) &&
            (fw->version == COG_VERSION) &&
            (cog_crc(fw) == fw->crc32) &&
            (fw->n_bins  == n_bins) &&
            (fw->en      == s_cbuf.en) &&
            (fw->zero_ref == s_cbuf.zero_ref) &&
            (fw->phase    == s_cbuf.phase) &&
            (fw->pp       == s_cbuf.pp) &&
            (memcmp(fw->cog, tab, (size_t)n_bins * sizeof(int16_t)) == 0)) {
            return 1U;
        }
    }

    if (ps_page_of_addr(COG_FLASH_ADDR, &bank, &page, &psize) == 0U) { return 0U; }

    s_ps_busy = 1U;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    memset(&er, 0, sizeof(er));
    er.TypeErase = FLASH_TYPEERASE_PAGES;
    er.Banks     = bank;
    er.Page      = page;
    er.NbPages   = 1U;
    if (HAL_FLASHEx_Erase(&er, &page_err) != HAL_OK) { ok = 0U; }

    if (ok != 0U) {
        const uint8_t *src = (const uint8_t *)&s_cbuf;

        for (i = 0U; i < (sizeof(CogBlob) / PS_DW); i++) {
            uint64_t dw = 0U;
            memcpy(&dw, src + (i * PS_DW), PS_DW);
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                  COG_FLASH_ADDR + (i * PS_DW), dw) != HAL_OK) {
                ok = 0U;
                break;
            }
        }
    }

    HAL_FLASH_Lock();
    s_ps_busy = 0U;

    return (ok != 0U) ? ParamStore_HasCog() : 0U;
}
