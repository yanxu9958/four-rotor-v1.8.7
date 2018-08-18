#include "fc_status.h"
#include "parse_packet.h"

_FC_STATUS fc = {0};

//是否锁定 状态轮询
void FC_status_polling(void)
{
    //解锁
    if(rc.thr<=10&&rc.yaw<=-25&&rc.pit>=25&&rc.rol<=-25)
        fc.unlock_cnt++;
    if(rc.thr<=10&&rc.yaw<=-25&&rc.pit>=25&&rc.rol<=-25&&fc.unlock_cnt>=600)
    {
        fc.unlock_cnt = 0;
        fc.state = fc_unlock;
        fc.unlock_flag = 1;                                 //解锁标志置位，指示灯使用该标志
    }
    //上锁
    if(rc.thr<=10&&rc.yaw>=25&&rc.pit>=25&&rc.rol>=25)
        fc.lock_cnt++;
    if(rc.thr<=10&&rc.yaw>=25&&rc.pit>=25&&rc.rol>=25&&fc.lock_cnt>=600)
    {
        fc.lock_cnt = 0;
        fc.state = fc_lock;
        fc.lock_flag = 1;                                   //上锁标志置位，指示灯使用该标志
    }
}



uint32_t  chip_id[3] = {0};  

//读取芯片ID
void get_chip_id(void)
{
    chip_id[0] = *(__IO u32 *)(0X1FFFF7F0); // 高字节
    chip_id[1] = *(__IO u32 *)(0X1FFFF7EC); //
    chip_id[2] = *(__IO u32 *)(0X1FFFF7E8); // 低字节
}









