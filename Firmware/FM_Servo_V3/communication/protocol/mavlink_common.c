/*
 * mavlink_common.c —— MAVLink v2 公共层实现（见 mavlink_common.h 设计说明）
 *
 * CRC 与组帧口径必须与 telemetry_tx.c / host/mavlink_proto.py 严格一致：
 *   CRC 覆盖 buf[1..9+len-1]（LEN 起，含 SEQ，不含 0xFD 魔数），末追加 crc_extra。
 * 历史坑：曾按含魔数口径组帧，pymavlink/QGC 全判 CRC 错（抓包 2134/2134 命中含魔数、
 * 0/2134 命中标准口径），见 Agnet/CRC口径与seq修正说明.md。此处采用标准口径。
 */
#include "mavlink_common.h"
#include <string.h>

/* 首期 9 条：len / crc_extra 来自 host/mavlink_layouts.json（2026-11-18 核对） */
static const MavMsgMeta_t s_meta[] = {
    { MAV_MSG_READ_PARAM,       4u,   43u },  /* READ_PARAM             */
    { MAV_MSG_APP_CONTROL_WORD, 3u,  108u },  /* AppControlWord         */
    { MAV_MSG_APP_OP_MODE,      1u,   26u },  /* AppOpMode              */
    { MAV_MSG_APP_PERMISSION,   2u,  134u },  /* AppPermissionConfig    */
    { MAV_MSG_SYSTEM_CMD,       1u,  169u },  /* SystemCmd              */
    { MAV_MSG_APP_STATUS_INFO, 46u,    3u },  /* AppStatusInfo          */
    { MAV_MSG_APP_MOTION_INFO,131u,   83u },  /* AppMotionInfo          */
    { MAV_MSG_HEARTBIT,         1u,   45u },  /* HeartBit               */
    { MAV_MSG_CUSTOM_INFO,      8u,    1u },  /* CustomInfo             */
    { MAV_MSG_POS_SPEED_CTL_IN,44u,    8u },  /* PosSpeedCtlInput       */
    { MAV_MSG_CURRENT_CTL_IN,  53u,  191u },  /* CurrentCtlInput        */
    { MAV_MSG_MIT_CTL_IN,      32u,   17u },  /* MitCtlInput            */
    { MAV_MSG_MIT_CTL_CONFIG,  16u,  147u },  /* MitCtlConfig           */
    { MAV_MSG_VEL_PROFILE_CFG,  8u,   89u },
    { MAV_MSG_APP_CTRL_STATUS, 40u,   90u },
    { MAV_MSG_APP_LOOP_STATUS, 48u,   91u },
    { MAV_MSG_APP_IDENT_RESULT, 40u,  92u },
    { MAV_MSG_APP_TORQUE_CMD,   12u,   95u },  /* 力矩模式 id / 斜坡（替代 '/' 与 '"'） */
    { MAV_MSG_APP_ADV_CFG,      24u,   96u },  /* 速度环陷波配置（crc_extra = msgid 低 8 位） */
    { MAV_MSG_APP_SPEED_TRAJ,   32u,   97u },  /* 速度轨迹到达时间优先（同上） */
    { MAV_MSG_APP_ENC_COMP,    136u,   98u },  /* 编码器角度软件补偿（16 阶谐波，带回读） */
    { MAV_MSG_APP_POS_STATUS,   48u,  100u },  /* 位置环状态（位置/目标/误差/状态机，只发不收） */
    { MAV_MSG_APP_COG_TABLE,    48u,   93u },  /* 齿槽表分块回读（替代 8 行 [MI] CG） */
    { MAV_MSG_APP_RECORD,      144u,   94u },  /* 录波（替代 '&' 与 '\''） */  /* 30090 状态字，新增必须同步登记，
                                               * 否则 MavMsg_Find 返回 0 → crc_extra=0
                                               * → 帧被判 CRC 错（30082 踩过） */  /* VelProfileConfig(自定义) */
    /* 2026-11-20：30082 必须在这里登记，否则 MavMsg_Find 返回 0 →
     * CRC 的 crc_extra 取 0（正确值 218）→ 192 B 的调参帧会被判 CRC 错
     * 静默丢弃，现象是"读得出来、写不进去"（实测踩过）。 */
    { MAV_MSG_APP_DEBUG_PARAM, 192u, 218u },  /* AppDebugParam          */
};
#define MAV_META_N  (sizeof(s_meta) / sizeof(s_meta[0]))

const MavMsgMeta_t *MavMsg_Find(uint32_t msgid)
{
    uint16_t i;
    for (i = 0u; i < MAV_META_N; i++) {
        if (s_meta[i].msgid == msgid) { return &s_meta[i]; }
    }
    return (const MavMsgMeta_t *)0;
}

uint16_t MavCrc_Calc(const uint8_t *p, uint16_t n, uint8_t extra)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    for (i = 0u; i < n; i++) {
        uint8_t t = (uint8_t)(p[i] ^ (uint8_t)(crc & 0xFFu));
        t ^= (uint8_t)(t << 4);
        crc = (uint16_t)((crc >> 8) ^ ((uint16_t)t << 8) ^
                         ((uint16_t)t << 3) ^ ((uint16_t)t >> 4));
    }
    {
        uint8_t t = (uint8_t)(extra ^ (uint8_t)(crc & 0xFFu));
        t ^= (uint8_t)(t << 4);
        crc = (uint16_t)((crc >> 8) ^ ((uint16_t)t << 8) ^
                         ((uint16_t)t << 3) ^ ((uint16_t)t >> 4));
    }
    return crc;
}

uint16_t MavFrame_Build(uint8_t *out, uint32_t msgid, uint8_t seq,
                        uint8_t sysid, uint8_t compid,
                        const uint8_t *payload, uint8_t len, uint8_t extra)
{
    uint16_t crc;

    out[0] = MAV_STX;
    out[1] = len;
    out[2] = 0u;                              /* incompat */
    out[3] = 0u;                              /* compat */
    out[4] = seq;
    out[5] = sysid;
    out[6] = compid;
    out[7] = (uint8_t)(msgid & 0xFFu);
    out[8] = (uint8_t)((msgid >> 8) & 0xFFu);
    out[9] = (uint8_t)((msgid >> 16) & 0xFFu);
    if ((payload != 0) && (len != 0u)) {
        memcpy(&out[10], payload, len);
    }
    /* CRC 覆盖 buf[1] 起 9+len 字节（含 SEQ、不含 0xFD），末追加 crc_extra */
    crc = MavCrc_Calc(&out[1], (uint16_t)(9u + len), extra);
    out[10u + len] = (uint8_t)(crc & 0xFFu);
    out[11u + len] = (uint8_t)(crc >> 8);

    return (uint16_t)(12u + len);
}
