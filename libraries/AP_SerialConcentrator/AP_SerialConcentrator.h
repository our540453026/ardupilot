/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <stack>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>
#define RESULT_ACCEPTED 0x01
#define RESULT_REJECTED 0x02
#define RESULT_FAILED   0x00
#define MR72_NO_ACTION  0x00
#define MR72_ACTION  0x01
#define MR72_DISABLE  0x02
#define MR72_ENABLE  0x03
#define MR72_OFF	0x04
#define UAV_NO_ACTION 0x00
#define UAV_CTR_GO_HOME  0x01
#define UAV_CTR_LAND	0x02
#define UAV_CTR_TAKEOFF 0x03
#define UAV_CTR_INIT_PPK 0x04
#define UAV_CTR_SHUTTER  0x05
#define UAV_CTR_GO_MISSION  0x06
#define UAV_CTR_MR72_OFF  0x07
#define UAV_MISSION_START 0x06
#define UAV_SHUI_WEN 0x07
#define UAV_TYPE 0x01
#define UAV_MISSION_TYPE_TILE 0x01
#define UAV_MISSION_TYPE_SHUIWEN 0x02
#define UAV_MISSION_TYPE_FANGDI 0x03
#define UAV_CTR_MSG_ID   0x0001
#define UAV_ACTIVATE_STATUE_MSG_ID   0x0002
#define UAV_MISSION_START_MSG_ID   0x0003
#define UAV_DO_SET_MISSION_CURRENT_MSG_ID   0x0004
#define UAV_DOWNLAOD_MISSION_CURRENT_MSG_ID   0x0005
#define UAV_MISSION_STATUE_CURRENT_MSG_ID   0x0006
#define UAV_STATUE1_CURRENT_MSG_ID   0x0007
#define UAV_HEARTBEAT_MSG_ID   0x0008
#define UAV_GPS_RAW_MSG_ID   0x0009
#define UAV_GPS_POS_MSG_ID   0x000A
#define UAV_RTK_RAW_MSG_ID   0x000B
#define UAV_ABNORMAL_MSG_ID   0x000C
#define UAV_GET_HOME_GPS_MSG_ID 0x000D
#define UAV_AVOID_MSG_ID   0x000F
#define UAV_TEMP_MSG_ID   0x0011
#define UAV_ESC_MSG_ID     0x000E
#define BMS_TYPE 	0x02
#define BMS_STATUE_MSG_ID   0x0101
#define BMS_STATUE2_MSG_ID   0x0104
#define BMS_ABNORMAL_MSG_ID   0x0102
#define CAMERA_TYPE 0x03
#define CAMERA_SHUTTER_MSG_ID   0x0201
#define CAMERA_CTR_MSG_ID   0x0202
#define CAMERA_STATUE_MSG_ID   0x0203
#define CAMERA_ABNORMAL_MSG_ID   0x0204
#define CAMERA_GIM_CMD_MSG_ID   0x0207
#define CAMERA_GIM_CMD2_MSG_ID   0x0208
#define CAMERA_GIM_ST_MSG_ID   0x020A
#define CAMERA_CTR_OFF	0x01
#define CAMERA_CTR_ON	0x02
#define CAMERA_CTR_REPAIR	0x03
#define CAMERA_CTR_SPEED_ADD	0x04
#define CAMERA_CTR_SPEED_LESS	0x05
#define CAMERA_CTR_SHUTTER	0x06
#define CAMERA_CTR_RESET_SPEED	0x07
#define CAMERA_CTR_START_RECODER	0x08
#define CAMERA_CTR_STOP_RECODER	0x09
#define CAMERA_CTR_ZOOM	0x1C
#define OPENMV_STATUE_ID_MSG_ID   0x0401
#define COMP_TYPE 0x00
#define COMP_GET_VER_MSG_ID   0x0f01
#define COMP_FIRMWARE_UPGRADE_MSG_ID   0x0f02
#define COMP_FIRMWARE_CODE_MSG_ID   0x0f03
#define COMP_GET_ID_MSG_ID   0x0f04
#define COMP_VALID_PERID_MSG_ID   0x0f05
#define COMP_GET_VALID_PERID_MSG_ID   0x0f06
#define COMP_PASSTHROUGH_MSG_ID   0x0f07
#define COMP_COPY_FLASH_MSG_ID 0x0f08
#define COMP_FLASH_MSG_ID		0x0f09
#define COMP_SYS_TIME_MSG_ID  0x0f0a
#define ACK_MSG_ID  0xFEFE
#define LINK_LENGTH 249
class AP_SerialConcentrator : public AP_ESC_Telem_Backend{




public:
    struct PACKED gimbal_st_t
    {
            uint32_t id;
            float pitch; /*单位 度 */	
            float roll;/*单位 度 */
            float yaw; /*单位 度 */          
    };
    struct PACKED gimbal_t
        {
            uint32_t id;
            uint8_t cam_type; /*<  0x01--可见光 0x02--红外*/
            uint8_t cam_cmd; /*< 0x01--拍照0x02--放大 0x03--缩小 0x04--开始录像 05--停止录像*/
            uint16_t cam_param; /*<相机参数*/		
            uint16_t gim_cmd; /*< 0x01--俯仰0x01--横滚 0x03--航向0x04--航向与俯仰 0x05--横滚与俯仰 0x06--横滚与俯仰与航向*/	
            float pitch; /*单位 度 */	
            float roll;/*单位 度 */
            float yaw; /*单位 度 */   
            uint16_t Cam_num;
            uint16_t Recoder_num;       
        };
    struct PACKED gimbal_ack_t
        {
            uint16_t ack_cmd;
            uint8_t cam_result; /*<  0x01--正常接收*/
            uint8_t gim_result; /*< 0x01--正常接收*/
                 
        };
        struct PACKED openmv_t
        {
            bool active;  //0--没有数据  1--有数据
            uint16_t type;
            uint16_t id;
            uint16_t x;
            uint16_t y; /*<  0x01--正常接收*/
            uint16_t r; /*< 0x01--正常接收*/ 
            uint32_t ipx_x_d;
            uint32_t ipx_y_d;
            uint32_t last_update_time;  
        };
        struct PACKED openmv_parse_t
        {
            uint16_t type;
            uint16_t id;
            uint16_t x;
            uint16_t y; /*<  0x01--正常接收*/
            uint16_t r; /*< 0x01--正常接收*/ 
            uint32_t ipx_x_d;
            uint32_t ipx_y_d; 
        }; 
    struct PACKED esc_st_t
        {
            uint16_t vot1; /*单位 V */ 
            uint16_t cur1;/*单位 A */
            uint16_t rpm1;
            int16_t tem1; /*单位 度 */
            uint16_t vot2; /*单位 V */ 
            uint16_t cur2;/*单位 A */
            uint16_t rpm2;
            int16_t tem2; /*单位 度 */
            uint16_t vot3; /*单位 V */ 
            uint16_t cur3;/*单位 A */
            uint16_t rpm3;
            int16_t tem3; /*单位 度 */
            uint16_t vot4; /*单位 V */ 
            uint16_t cur4;/*单位 A */
            uint16_t rpm4;
            int16_t tem4; /*单位 度 */	         
        };
    struct PACKED bms_t
        {
            uint32_t id;
            uint32_t time_s;	
            uint16_t voltages; /*<  Type voltage of the battery*/
            uint16_t cell_vot[15]; /*< [mV] Battery voltage of cells. Cells above the valid cell count for this battery should have the UINT16_MAX value.*/
            uint16_t current_battery; /*< [mA] Battery current, -1: autopilot does not measure the current*/		
            uint16_t capacity_full; /*< [mAh] full capacity of the battery..*/	
            uint16_t capacity_remaining; /*< [mAh]  remaining capacity of the battery..*/	
            uint16_t cycle;
            uint16_t charge_count; /*<  charge of count*/
            uint16_t discharge_count; /*<  discharge of count.*/	
            int8_t	bat_temp1;
            int8_t 	bat_temp2;
            int8_t 	pcb_temp1;
            int8_t 	pcb_temp2;
            uint8_t type;                    // stage failsafe the battery is in

        };
    struct PACKED bms2_t
        {
            uint32_t id;
            uint32_t time_s;	
            uint16_t voltages; /*<  Type voltage of the battery*/
            uint16_t cell_vot[18]; /*< [mV] Battery voltage of cells. Cells above the valid cell count for this battery should have the UINT16_MAX value.*/
            uint16_t current_battery; /*< [mA] Battery current, -1: autopilot does not measure the current*/		
            uint16_t capacity_full; /*< [mAh] full capacity of the battery..*/	
            uint16_t capacity_remaining; /*< [mAh]  remaining capacity of the battery..*/	
            uint16_t cycle;
            uint16_t charge_count; /*<  charge of count*/
            uint16_t discharge_count; /*<  discharge of count.*/	
            int8_t	bat_temp1;
            int8_t 	bat_temp2;
            int8_t 	pcb_temp1;
            int8_t 	pcb_temp2;
            uint8_t type;                    // stage failsafe the battery is in
        };
    struct PACKED bms_staute_t
        {
            uint32_t id;
            uint32_t time_s;	
            uint32_t voltages; /*<  Type voltage of the battery*/
            uint16_t cell_vot[18]; /*< [mV] Battery voltage of cells. Cells above the valid cell count for this battery should have the UINT16_MAX value.*/
            uint32_t current_battery; /*< [mA] Battery current, -1: autopilot does not measure the current*/		
            uint32_t capacity_full; /*< [mAh] full capacity of the battery..*/	
            uint32_t capacity_remaining; /*< [mAh]  remaining capacity of the battery..*/	
            int8_t	bat_temp;
        };
        
    uint32_t        _last_update_time;
    bool    Tx_urgent_statue = false;
    typedef struct
        {
            uint16_t  front;
            uint16_t  rear;
            uint8_t buf[LINK_LENGTH];

        }LinkMes;
    LinkMes usart_rx_fifo;

    AP_SerialConcentrator();
    void link_init(void);
    void InitMesQueue(LinkMes *L);
    char EnMesQueue(LinkMes *L,uint8_t dat);
    char DeMesQueue(LinkMes *L , uint8_t *dat);
    uint8_t usart_serial_available(uint8_t *dat);
   // ~AP_SerialConcentrator();

    /* Do not allow copies */
  //  AP_SerialConcentrator(const AP_SerialConcentrator &other) = delete;
    //AP_SerialConcentrator &operator=(const AP_SerialConcentrator&) = delete;

    // init - perform required initialisation
    bool init(const AP_SerialManager &serial_manager);
    void send_data(uint8_t *buffer,uint8_t size);
    void send_gimbal_data(gimbal_t gimbal);
    void send_gimbal_st_data(gimbal_st_t gimbal);
    bool update(void);
    void send_camare_record_on(void);
    void send_camare_record_off(void);
    void send_camare_ctl(uint8_t cmd, uint32_t id,uint32_t param);
    bms_staute_t bms_state;
    gimbal_t gimbal_state;
    gimbal_st_t gimbal_st_u;
    openmv_t openmv_data[8];

    static AP_SerialConcentrator *get_singleton(void) { return _singleton; }
private:
    uint16_t g_crc_val,test_counter;
    uint32_t swapInt32( uint32_t value);
    uint16_t swapInt16( uint16_t value);
    uint16_t swapuInt16( uint16_t value);
    float swap_fl_bits(float value) ;
    uint8_t delay_n,dug,gimbal_hz,resend_gimbal_hz,test_flag;
    int8_t pcbtemp[4];
    void update_tx_buf(void);
    bool Data_Message_Process(void);
    void update_disconnect(void);
    uint16_t crcSum(uint8_t *dat,uint8_t n);
    bool parse(uint8_t temp);
    static const uint8_t DATA_PREAMBLE1 = 0xAB;
    static const uint8_t DATA_PREAMBLE2 = 0xCB;
    AP_HAL:: UARTDriver *_port;
    static AP_SerialConcentrator *_singleton;
    union PACKED msgbuffer {
                bms_t bms;
                bms2_t bms2;
                gimbal_ack_t gimbal_ack_u;
                gimbal_st_t gimbal_st_u;
                esc_st_t esc_st;
                openmv_parse_t openmv_u;
                uint8_t bytes[256];
            };
    struct PACKED data_msg_parser
        {
            enum
            {
                PREAMBLE1 = 0,
                PREAMBLE2,
                LENGTH1,
                LENGTH2,
                COMPTYPE,
                MSGID1,
                MSGID2,
                PAYLOAD,
                CRC1,
                CRC2,
            } rec_state;
            uint16_t length;
            uint16_t msgid;
            uint8_t comptype;
            msgbuffer data;
            uint16_t crc;
            uint16_t read_count;
        } data_msg;

};

namespace AP {
AP_SerialConcentrator *serialConcentrator();
};
