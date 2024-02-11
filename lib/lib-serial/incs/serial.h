/*!
 * @file: serial.h
 * @date: 2024-02-16
 * @author: Sebastien CORBEAU (corbeau.sebastien@yahoo.fr)
 * @brief: Declaration of function to manage serial port.
 */
#ifndef LIB_SERIAL_SERIAL_H__
#define LIB_SERIAL_SERIAL_H__

#include <stdbool.h>

/*!
 * @struct serial_handle_t
 * @brief Handle of UART port.
 */
typedef struct serial_handle_t serial_handle_t;

/*!
 * @enum serial_parity_bit_t
 * @brief Serial parity.
 */
typedef enum serial_parity_bit_t
{
    SERIAL_PARITY_NO    = 0,    /*!< No parity. */
    SERIAL_PARITY_ODD   = 1,    /*!< Parity Odd. */
    SERIAL_PARITY_EVEN  = 2,    /*!< Parity Even. */
    SERIAL_PARITY_MARK  = 3,    /*!< Parity Mark. */
    SERIAL_PARITY_SPACE = 4,    /*!< Parity Space. */
    SERIAL_PARITY_NB_VALUE,     /*!< Number of parity value. */
} serial_parity_bit_t;

/*!
 * @enum serial_stop_bits_t
 * @brief Serial number of stop bit(s).
 */
typedef enum serial_stop_bits_t
{
    SERIAL_STOP_BIT_1   = 1,    /*!< One stop bit. */
    SERIAL_STOP_BIT_2   = 2,    /*!< Two stop bits. */
    SERIAL_STOP_BIT_1_5 = 3,    /*!< 1.5 stop bits. */
    SERIAL_STOP_NB_VALUE,       /*!< Number of serial stop bits value. */
} serial_stop_bits_t;

/*!
 * @enum serial_stop_bits_t
 * @brief Serial number of stop bit(s).
 */
typedef enum serial_hardware_flow_control_t
{
    SERIAL_NO_HW_FLOW_CTRL  = 0,    /*!< No flow control. */
    SERIAL_RTS_CTS          = 1,    /*!< RTS/CTS flow control. */
    SERIAL_DTR_DSR          = 2,    /*!< DTR/DSR flow control. */
    SERIAL_HW_CTRL_BOTH     = 3,    /*!< RTS/CTS and DTR/DSR flow control. */
    SERIAL_NB_HW_FLOW_CTRL,         /*!< Number of flow control. */
} serial_hardware_flow_control_t;

/*!
 * @brief Open serial port and init it.
 * @param[in]   port_name       Name of port (In Windows COMx:, In Linux 
 *                              /dev/ttyUSBx)
 * @param[in]   baudrate        Baudrate of serial port.
 * @param[in]   nb_data_bits    Number of data bits (value between 5 and 8).
 * @param[in]   parity_bit      Parity bit.
 * @param[in]   stop_bit        Number of stop bit (When use 5 data bits with 2 
 *                              stop bits is invalid, like 6, 7 or 8 data bits 
 *                              with 1.5 stop bits ).
 * @param[in]   flow_ctrl       Flow control.
 * @return Handle to serial manager on success otherwise NULL.
 */
struct serial_handle_t * open_serial_port( const char * port_name,
        uint32_t baudrate,
        uint8_t nb_data_bits,
        serial_parity_bit_t parity_bit,
        serial_stop_bits_t stop_bit,
        serial_hardware_flow_control_t flow_ctrl );

/*!
 * @brief Close serial port and free allocated resource.
 * @param[in]   serial  Handle on serial.
 * @return 0 on success otherwise -1.
 */
int close_serial_port( struct serial_handle_t * serial );

/*!
 * @brief Check if input data from serial is available.
 * @param[in]   serial  Handle on serial.
 * @return true when data available otherwise false.
 */
bool is_serial_data_available( struct serial_handle_t * serial );

/*!
 * @brief Get available data from serial (if all datas required are 
 *        not available function return the number of read).
 * @param[in]       serial  Handle on serial.
 * @param[out]      buffer  Pointer on buffer to store data.
 * @param[inout]    size    Pointer on uint32_t storing data required in input
 *                          and contains data read as output.
 * @return -1 on error the number of data read in success (0 when no data ).
 */
int get_serial_data_non_blocking( struct serial_handle_t * serial,
                                  uint8_t * buffer,
                                  uint32_t * size );

/*!
 * @brief Get available data from serial (wait that all datas are received 
 *        except if driver closing is required).
 * @param[in]       serial  Handle on serial.
 * @param[out]      buffer  Pointer on buffer to store data.
 * @param[inout]    size    Pointer on uint32_t storing data required in input
 *                          and contains data read as output.
 * @param[in]       timeout Timeout in milli-seconds.
 * @return -1 on error the number of data read in success (0 when no data ).
 */
int get_serial_data_blocking( struct serial_handle_t * serial,
                                  uint8_t * buffer,
                                  uint32_t * size,
                                  uint32_t timeout );

/*!
 * @brief Send data to serial.
 * @param[in]   serial  Handle on serial.
 * @param[in]   data    Pointer on buffer with data to send.
 * @param[in]   size    Size of data to send.
 * @return -1 on error the number of data send in success.
 */
int send_data_to_serial( struct serial_handle_t * serial,
                         const uint8_t * data,
                         uint32_t size );

#endif /* LIB_SERIAL_SERIAL_H__ */