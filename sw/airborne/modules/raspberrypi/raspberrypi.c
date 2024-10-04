#include "modules/raspberrypi/raspberrypi.h"
#include "pprzlink/pprz_transport.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include <time.h>
#include <sys/time.h>
#include "modules/core/abi.h"
#include <stdio.h> 

static abi_event RPI_out;
uint8_t sending_msg_id;
struct raspberrypi_data_in rpi_data_in;
struct raspberrypi_data_out rpi_value;
struct raspberrypi_data_out rpi_data_out;
//float extra_data_in[255]__attribute__((aligned));
uint16_t buffer_in_counter = 0;
uint32_t missed_packets = 0;
uint16_t rpi_message_frequency_RX = 0;
uint32_t received_packets = 0;
uint32_t send_counter=0;
float last_ts = 0;
static uint8_t rpi_msg_buf_in[sizeof(struct raspberrypi_data_in)*2]  __attribute__((aligned));

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
static void send_pixel_data(struct transport_tx *trans, struct link_device *dev)
{
  //float pos_x_f = rpi_data_in.pos_x_int*0.1f;
  //float pos_y_f = rpi_data_in.pos_y_int*0.1f; 
  //float pos_target_telemetry[2] = {rpi_data_in.pos_x_int*0.1f, rpi_data_in.pos_y_int*0.1f} 
  pprz_msg_send_RASPBERRY(trans, dev, AC_ID,
                              &rpi_data_in.pos_x_int,
                              &rpi_data_in.pos_y_int);
}
#endif

static void data_rpi_out(uint8_t sender_id __attribute__((unused)), struct raspberrypi_data_out * rpi_data_out_ptr){

    memcpy(&rpi_data_out,rpi_data_out_ptr,sizeof(struct raspberrypi_data_out));
    //memcpy(&extra_data_out,extra_data_out_ptr, sizeof(extra_data_out) );

    //Increase the counter to track the sending messages:
    //myam7_data_out.rolling_msg_out = extra_data_out[sending_msg_id];
    //myam7_data_out.rolling_msg_out_id = sending_msg_id;
    //sending_msg_id++;
    // if(sending_msg_id == 255){
    //     sending_msg_id = 0;
    // }

    //Send the message over serial to the Raspberry pi:
    uint8_t *buf_send = (uint8_t *)&rpi_data_out;
    //Calculating the checksum
    uint8_t checksum_out_local = 0;
    for(uint16_t i = 0; i < sizeof(struct raspberrypi_data_out) - 1; i++){
        checksum_out_local += buf_send[i];
    }
    rpi_data_out.checksum_out = checksum_out_local;
    //Send bytes
    uart_put_byte(&(RPI_PORT), 0, START_BYTE);
    for(uint8_t i = 0; i < sizeof(struct raspberrypi_data_out) ; i++){
        printf("sent");
        uart_put_byte(&(RPI_PORT), 0, buf_send[i]);
    }
}

void raspberrypi_init() 
{
    buffer_in_counter = 0;
    sending_msg_id = 0;
    rpi_data_in.yaprak = 2000;
    AbiSendMsgRPI_INFO(RPI_INFO_ID, &rpi_data_in);
    // //Init abi bind msg:
    AbiBindMsgRASPBERRY_OUT(ABI_BROADCAST, &RPI_out, data_rpi_out);

    #if PERIODIC_TELEMETRY
    // register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_AM7_IN, am7_downlink);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RASPBERRY, send_pixel_data);
    #endif
}

// void raspberrypi_periodic(void)
// {
//     rpi_value.iteration=1;
    
// }
/* Parse the InterMCU message */
void rpi_parse_msg_in(void)
{
    
    memcpy(&rpi_data_in, &rpi_msg_buf_in[1], sizeof(struct raspberrypi_data_in));
    //Assign the rolling message:
    //extra_data_in[rpi_data_in.rolling_msg_in_id] = rpi_data_in.rolling_msg_in;
    //Send msg through ABI:
    //AbiSendMsgRASPBERRY(ABI_AM7_DATA_IN_ID, &rpi, &extra_data_in[0]);
}

/* We need to wait for incoming messages */
void raspberrypi_event()
{
    //printf("uart_char_available: %d\n", uart_char_available(&(RPI_PORT)));
    if(fabs(get_sys_time_float() - last_ts) > 5){
        received_packets = 0;
        last_ts = get_sys_time_float();
    }
    while(uart_char_available(&(RPI_PORT)) > 0) {
        
        uint8_t rpi_byte_in;
        rpi_byte_in = uart_getch(&(RPI_PORT));
        if ((rpi_byte_in == START_BYTE) || (buffer_in_counter > 0)) {
            //printf("yarrr");
            rpi_msg_buf_in[buffer_in_counter] = rpi_byte_in;
            buffer_in_counter++;
        }
        if (buffer_in_counter > sizeof(struct raspberrypi_data_in) ) {
            buffer_in_counter = 0;
            uint8_t checksum_in_local = 0;
            for(uint16_t i = 1; i < sizeof(struct raspberrypi_data_in) ; i++){
                checksum_in_local += rpi_msg_buf_in[i];
            }
            printf("checksum local: %d, checksum global %d\n", checksum_in_local, rpi_msg_buf_in[sizeof(struct raspberrypi_data_in)]);
            if(checksum_in_local == rpi_msg_buf_in[sizeof(struct raspberrypi_data_in)]){
                rpi_parse_msg_in();
                received_packets++;
                AbiSendMsgRPI_INFO(RPI_INFO_ID, &rpi_data_in);
                //rpi_value.iteration = 1;
                //data_rpi_out(&rpi_value);
                
                //printf("R\n");
                //printf("pos_x_int: %d, pos_y_int: %d\n", rpi_data_in.pos_x_int, rpi_data_in.pos_y_int);
            }
            else {
                //printf("N\n");
                missed_packets++;
            }
        }
    }
    rpi_message_frequency_RX = (uint16_t) received_packets/(get_sys_time_float() - last_ts);
}