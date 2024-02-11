/*!
 * @file: serial.c
 * @date: 2024-02-26
 * @author: Sebastien CORBEAU (corbeau.sebastien@yahoo.fr)
 * @brief: Implementation of function to manage serial port.
 */
#include <windows.h>

#include <lib-utils-assert.h>
#include <lib-utils-fifo.h>
#include <lib-utils-logger.h>

#include "serial.h"

/*!
 * @brief Size of RX FIFO
 */
#define RX_FIFO_SIZE    1024

/*!
 * @brief Size of TX FIFO
 */
#define TX_FIFO_SIZE    1024

/*!
 * @brief Struct to store serial port information.
 * @enum serial_event_id_t
 */
typedef enum serial_event_id_t
{
    POWER_DOWN_EVT = 0,     /*!< Power down event. */
    LAUNCH_READ_EVT = 1,    /*!< Launch reader process event. */
    READ_DONE_EVT = 2,      /*!< Data is received event. */
    LAUNCH_WRITER_EVT = 3,  /*!< Launch writer process event. */
    WRITE_DONE_EVT = 4,     /*!< Data are send event. */
    NUMBER_EVT,             /*!< Number of event. */ 
} serial_event_id_t;

/*!
 * @brief Struct to store serial port information.
 * @struct serial_handle_t
 */
typedef struct serial_handle_t
{
    HANDLE handle;  /*!< Handle to manipulate serial port. */
    HANDLE events[NUMBER_EVT];   /*!< Event to manage receiver/sender thread. */
    HANDLE thread;  /*!< Handle of serial thread. */
    DWORD  th_id;   /*!< Thread identifier. */
    DECL_FIFO(rx, uint8_t, RX_FIFO_SIZE);   /*!< RX FIFO. */
    DECL_FIFO(tx, uint8_t, TX_FIFO_SIZE);   /*!< TX FIFO. */
    CRITICAL_SECTION tx_mutex;              /*!< TX FIFO protect. */
    CRITICAL_SECTION rx_mutex;              /*!< RX FIFO protect. */
    bool tx_process;                        /*!< Flag for tx process. */
} serial_handle_t;

/*!
 * @brief Check and configure DCB.
 * @param[inout]    serial          Handle of serial port.
 * @param[in]       baudrate        Baudrate of serial port.
 * @param[in]       nb_data_bits    Number of data bits (value between 5 and 8).
 * @param[in]       parity_bit      Parity bit.
 * @param[in]       stop_bit        Number of stop bit (When use 5 data bits 
 *                                  with 2 stop bits is invalid, like 6, 7 or 8 
 *                                  data bits with 1.5 stop bits ).
 * @param[in]       flow_ctrl       Flow control.
 * @return 0 on success otherwise -1.
 */
static int configure_dcb( HANDLE serial,
                          uint32_t baudrate,
                          uint8_t nb_data_bits,
                          serial_parity_bit_t parity_bit,
                          serial_stop_bits_t stop_bit,
                          serial_hardware_flow_control_t flow_ctrl );

/*!
 * @brief Initialize event to manage serial port.
 * @param[inout]   serial   Handle of serial port.
 * @return 0 on success otherwise -1.
 */
static int init_event( struct serial_handle_t * serial );

/*!
 * @brief Serial thread process.
 * @param[in]   lpParameter     Thread parameter (pointer on serial_handle_t).
 * @return 0 on success -1 when thread error.
 */
static DWORD WINAPI ThreadSerial( LPVOID lpParameter );

struct serial_handle_t * open_serial_port( const char * port_name,
        uint32_t baudrate,
        uint8_t nb_data_bits,
        serial_parity_bit_t parity_bit,
        serial_stop_bits_t stop_bit,
        serial_hardware_flow_control_t flow_ctrl  )
{
    struct serial_handle_t * ret = NULL;
    COMMTIMEOUTS timeout = {0, 0, 0, 0, 0};
    bool error = false;

    ASSERT_STR_NOT_NULL(port_name, NULL);
    ASSERT_UINT32(baudrate, 110, 256000, NULL);
    ASSERT_UINT8(nb_data_bits, 5, 8, NULL);
    ASSERT_UINT8(parity_bit, 0, SERIAL_PARITY_NB_VALUE, NULL);
    ASSERT_UINT8(stop_bit, 0, SERIAL_STOP_NB_VALUE, NULL);
    ASSERT_UINT8(flow_ctrl, 0, SERIAL_NB_HW_FLOW_CTRL, NULL);

    ret = malloc( sizeof(*ret) );
    if( ! ret )
    {
        ERROR_LOG("Allocate memory");
        error = true;
    }
    else
    {
        ret->thread = NULL;
        ret->handle = CreateFile( port_name, \
                                  GENERIC_READ | GENERIC_WRITE, \
                                  0, \
                                  NULL, \
                                  OPEN_EXISTING, \
                                  FILE_FLAG_OVERLAPPED, \
                                  NULL); 
        if( INVALID_HANDLE_VALUE == ret->handle )
        {
            ERROR_LOG( "Failed to open %s",  port_name );
            error = true;
        }
    }

    if( ! error )
    {
        if( 0 != configure_dcb( ret,
                                baudrate, 
                                nb_data_bits,
                                parity_bit,
                                stop_bit, 
                                flow_ctrl ) )
        {
            error = true;
        }

        timeout.ReadIntervalTimeout = 0;
        timeout.ReadTotalTimeoutMultiplier = 0;
        timeout.ReadTotalTimeoutConstant = 0;
        timeout.WriteTotalTimeoutMultiplier = 0;
        timeout.WriteTotalTimeoutConstant = 0;

        if( 0 == SetCommTimeouts(serial, &timeout) )
        {
            ERROR_LOG("Failed to set Comm Timeouts");
            error = true;
        }

        if( 0 == SetCommMask(ret, 0) )
        {
            ERROR_LOG("Failed to set Comm Mask");
            error = true;
        }
    }

    if( ! error )
    {
        if( -1 == init_event  (&ret ))
        {
            error = true;
        }
    }

    if( ! error )
    {
        INIT_FIFO( &ret->rx );
        INIT_FIFO( &ret->tx );

        InitializeCriticalSection( &ret->rx_mutex );
        InitializeCriticalSection( &ret->tx_mutex );

        ret->tx_process = FALSE;

        ret->thread = CreateThread( NULL,
                                    0,  
                                    ThreadSerial,
                                    ret, 
                                    0, 
                                    &ret->th_id );

        if( NULL == ret->thread )
        {
            error = true;
        }
    }

    if( error )
    {
        if(ret)
        {
            if( NULL != ret->thread )
            {
                SetEvent( ret->events[POWER_DOWN_EVT] );
                WaitForSingleObject( ret->thread, INFINITE);
            }
            
            for(i=0; i<NUMBER_EVT; i++)
            {
                if( NULL != ret->events[i] )
                {
                    CloseHandle( ret->events[i] );
                }
            }

            if( INVALID_HANDLE_VALUE != ret->handle )
            {
                CloseHandle( ret->handle );
            }

            free(ret);

            ret = NULL;
        }
    }

    return ret;
}

int close_serial_port( struct serial_handle_t * serial )
{
    ASSERT_PTR(serial, -1);

    /* Signal to thread to stop */
    SetEvent( serial->events[POWER_DOWN_EVT] );

    /* Wait thread death. */
    WaitForSingleObject( serial->thread, INFINITE);

    /* Free event */
    for(i=0; i<NUMBER_EVT; i++)
    {
        CloseHandle( serial->events[i] );
    }

    /* Destroy critical section */
    DeleteCriticalSection( &serial->tx_mutex );
    DeleteCriticalSection( &serial->rx_mutex );

    /* Close serial port */
    CloseHandle( serial->serial );

    free(serial);
    serial = NULL;

    return 0;
}

bool is_serial_data_available( struct serial_handle_t * serial )
{
    bool ret;

    ASSERT_PTR(serial, false);

    EnterCriticalSection( &serial->rx_mutex );
    ret = ! IS_FIFO_EMPTY( &serial->rx );
    LeaveCriticalSection( &serial->rx_mutex );

    return ret;
}

int get_serial_data_non_blocking( struct serial_handle_t * serial,
                                  uint8_t * buffer,
                                  uint32_t * size )
{
    bool isFifoEmpty;
    uint32_t i;

    ASSERT_PTR(serial, -1);
    ASSERT_PTR(buffer, -1);
    ASSERT_PTR(size, -1);
    ASSERT((*size <= 0), -1);

    EnterCriticalSection( &serial->rx_mutex );
    isFifoEmpty = IS_FIFO_EMPTY( &serial->rx );
    LeaveCriticalSection( &serial->rx_mutex );

    i = 0;

    while( ! isFifoEmpty && ( i < *size ) )
    {
        EnterCriticalSection( &serial->rx_mutex );
        buffer[i] = serial->rx.data[serial->rx.ptr_out];
        INC_PTR_OUT( &serial->rx );
        isFifoEmpty = IS_FIFO_EMPTY( &serial->rx );
        LeaveCriticalSection( &serial->rx_mutex );

        i++;
    }

    *size = i;

    return i;
}

int get_serial_data_blocking( struct serial_handle_t * serial,
                                  uint8_t * buffer,
                                  uint32_t * size,
                                  uint32_t timeout )
{
    int ret = -1;
    DWORD now;
    DWORD end;
    bool is_timeout = false;

    ASSERT_PTR(serial, -1);
    ASSERT_PTR(buffer, -1);
    ASSERT_PTR(size, -1);
    ASSERT((*size <= 0), -1);

    if( timeout )
    {
        now = GetTickCount( );
        end = now + timeout;
    }

    while( ! is_timeout && ( i < *size ) )
    {
        EnterCriticalSection( &serial->rx_mutex );
        if( IS_FIFO_EMPTY( &serial->rx ) )
        {
            buffer[i] = serial->rx.data[serial->rx.ptr_out];
            INC_PTR_OUT( &serial->rx );
            i++;
        }
        LeaveCriticalSection( &serial->rx_mutex );

        if( timeout )
        {
            now = GetTickCount( );

            if( now >= end )
            {
                is_timeout = true;
            }
        }
    }

    *size = i;

    return i;
}                                  

int send_data_to_serial( struct serial_handle_t * serial,
                         const uint8_t * data,
                         uint32_t size )
{
    int ret = -1;
    bool fifo_full;

    ASSERT_PTR( serial, -1);
    ASSERT_PTR( data, -1);
    ASSERT( ( (0 == size) || (size >= ( TX_FIFO_SIZE - 1 ) ) , -1);

    EnterCriticalSection( &serial->tx_mutex );
    fifo_full = IS_FIFO_FULL( serial->tx );
    LeaveCriticalSection( &serial->tx_mutex );

    while( ! fifo_full && (i < size ) )
    {
        EnterCriticalSection( &serial->tx_mutex );
        serial->tx[serial->tx->ptr_in] = data[i];

        INC_PTR_IN( serial->tx );

        fifo_full = IS_FIFO_FULL( serial->tx );
        LeaveCriticalSection( &serial->tx_mutex );
        
        i++;
    }
    
    EnterCriticalSection( &serial->tx_mutex );
    if( ! serial->tx_process )
    {        
        serial->tx_process = true;
        SetEvent( serial->events[LAUNCH_WRITER_EVT] );
    }
    LeaveCriticalSection( &serial->tx_mutex );

    return ret;
}

static int configure_dcb( HANDLE serial,
                          uint32_t baudrate,
                          uint8_t nb_data_bits,
                          serial_parity_bit_t parity_bit,
                          serial_stop_bits_t stop_bit,
                          serial_hardware_flow_control_t flow_ctrl )
{
    int ret = 0;
    DCB dcb;

    ASSERT( ( INVALID_HANDLE_VALUE == serial ), -1 );
    
    memset( &dcb, 0, sizeof( dcb ) );

    dcb.DCBlength = sizeof(dcb);
    dcb.fBinary   = TRUE;                   
    dcb.BaudRate  = baudrate;

    switch( parity_bit )
    {
        case SERIAL_PARITY_NO:
            dcb.Parity  = NOPARITY;
            dcb.fParity = FALSE;
        break;

        case SERIAL_PARITY_EVEN:
            dcb.Parity  = EVENPARITY;
            dcb.fParity = TRUE;
        break;

        case SERIAL_PARITY_ODD:
            dcb.Parity  = ODDPARITY;
            dcb.fParity = TRUE;
        break;

        case SERIAL_PARITY_MARK:
            dcb.Parity  = MARKPARITY;
            dcb.fParity = TRUE;
        break;

        case SERIAL_PARITY_SPACE:
            dcb.Parity  = SPACEPARITY;
            dcb.fParity = TRUE;
        break;

        default:
            ret = -1;
        break;
    }

    switch (stop_bit)
    {
        case SERIAL_STOP_BIT_1:
            dcb.StopBits    = ONESTOPBIT;
        break;

        case SERIAL_STOP_BIT_2:
            dcb.StopBits    = TWOSTOPBITS;
        break;

        case SERIAL_STOP_BIT_1_5:
            dcb.StopBits    = ONE5STOPBITS;
        break;
        
        default:
            ret = -1;
        break;
    }

    dcb.fOutxCtsFlow    = FALSE;
    dcb.fOutxDsrFlow    = FALSE;
    dcb.fDtrControl     = DTR_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fRtsControl     = RTS_CONTROL_DISABLE;
    if(( SERIAL_RTS_CTS == flow_ctrl ) || ( SERIAL_HW_CTRL_BOTH == flow_ctrl ))
    {
        dcb.fOutxCtsFlow    = TRUE;
        dcb.fRtsControl     = RTS_CONTROL_HANDSHAKE;
    }

    if((SERIAL_DTR_DSR == flow_ctrl) || ( SERIAL_HW_CTRL_BOTH == flow_ctrl ))
    {
        dcb.fOutxDsrFlow    = TRUE;
        dcb.fDtrControl     = DTR_CONTROL_DISABLE;
        dcb.fDsrSensitivity = FALSE;
    }
   
    dcb.ByteSize        = nb_data_bits;
    
    if((nb_data_bits == 5) && (stop_bit == SERIAL_STOP_BIT_2)
    {
        ret = -1;
    }

    if(SERIAL_STOP_BIT_1_5 == stop_bit )
    {
        if((nb_data_bits >=6) && (nb_data_bits<=8))
        {
            ret = -1;
        }
    }
       
    dcb.fOutX           = FALSE;
    dcb.fInX            = FALSE; 
    dcb.fErrorChar      = FALSE;
    dcb.fNull           = FALSE;
    dcb.fAbortOnError   = FALSE;
    dcb.XonLim          = 8;
    dcb.XoffLim         = 16;
    dcb.XonChar         = 0;
    dcb.XoffChar        = 0;
    dcb.EofChar         = 0;
    dcb.EvtChar         = 0;

    if( 0 == ret )
    {
        if( 0 == SetCommState( serial, &dcb ) )
        {
            ERROR_LOG("Failed to configure DCB");
            ret = -1;
        }
    }

    return ret;
}

static int init_event( struct serial_handle_t * serial )
{
    int ret = 0;
    uint8_t i;

    ASSERT( (NULL == serial), -1 );

    for( i=0; i<NUMBER_EVT; i++ )
    {
        serial->events[i] = NULL;
    }

    /* Automatic reset */
    serial->events[POWER_DOWN_EVT] = CreateEvent(NULL, FALSE, FALSE, NULL);
    
    serial->events[WRITE_DONE_EVT] = CreateEvent(NULL, FALSE, FALSE, NULL);

    /* Manuel reset */
    serial->events[READ_DONE_EVT] = CreateEvent(NULL, TRUE, FALSE, NULL);
    serial->events[LAUNCH_WRITER_EVT] = CreateEvent(NULL, TRUE, FALSE, NULL);
    
    for( i=0; i<NUMBER_EVT; i++)
    {
        if( NULL == serial->events[i] )
        {
            ret = -1;
        }
    }

    if( -1 == ret )
    {
        for( i=0; i<NUMBER_EVT; i++)
        {
            if( serial->events[i] )
            {
                CloseHandle( serial->events[i] );
            }
        }
    }

    return ret;
}

static DWORD WINAPI ThreadSerial( LPVOID lpParameter )
{
    struct serial_handle_t * serial = (struct serial_handle_t *) lpParameter;
    bool hasToContinue = true;
    DWORD waitObject;
    OVERLAPPED  ovRx;
    OVERLAPPED  ovTx;
    BOOL ret;
    DWORD nbByteRead;
    DWORD nbByteWritten;
    uint8_t tx_buff[ TX_FIFO_SIZE ];
    uint32_t tx_size;

    ASSERT( (NULL == serial), -1 );

    /* Launch first read. */
    ovRx.hEvent = serial->events[READ_DONE_EVT];
    ret = ReadFile( serial->serial,
                    &serial->rx.data[serial->rx.ptr_in],
                    1,
                    &nbByteRead,
                    &ovRx);

    ovWriter.hEvent = serial->events[LAUNCH_WRITER_EVT];
    if( ! ret )
    {
        hasToContinue = false;
    }

    while( hasToContinue )
    {
        waitObject = WaitForMultipleObjects( NUMBER_EVT, \
                                             serial->events, \
                                             FALSE,
                                             INFINITE );
        waitObject = waitObject - WAIT_OBJECT_0;

        switch ( waitObject )
        {
        case POWER_DOWN_EVT:
            haseToContinue = false;
        break;

        case READ_DONE_EVT:
            ret = GetOverlappedResult( serial->serial,
                                       &ovRx,
                                       &nbByteRead,
                                       FALSE); 
            if ( ret )
            {
                ResetEvent( serial->events[READ_DONE_EVT] );

                EnterCriticalSection( &serial->rx_mutex );
                if( IS_FIFO_FULL( &serial->rx ) )
                {
                    INC_PTR_OUT( &serial->rx );
                    /* TODO: Add flag to signal overflow. */
                }
                INC_PTR_IN( &serial->rx );
                LeaveCriticalSection( &serial->rx_mutex );

                ret = ReadFile( serial->serial,
                                &serial->rx.data[serial->rx.ptr_in],
                                1,
                                &nbByteRead,
                                &ovRx);
                if( ! ret )
                {
                    hasToContinue = false;
                }
            }
        break;

        case LAUNCH_WRITER_EVT:
            tx_size = prepare_next_tx_xfer( serial, tx_buff, TX_FIFO_SIZE );

            if( tx_size )
            {
                /* Write file. */
                ret = WriteFile( serial->serial, \
                                 tx_buff, \
                                 tx_size, \
                                 NULL, \
                                 &ovTx);
                if( ! ret && ( ERROR_IO_PENDING != GetLastError() )
                {
                    hasToContinue = false;
                }
            }
        break;
    
        case WRITE_DONE_EVT:
            ret = GetOverlappedResult( serial->serial,
                                       &ovTx,
                                       &nbByteWritten,
                                       FALSE ); 
            if ( ret )
            {
                ResetEvent( serial->events[WRITE_DONE_EVT] );
                
                tx_size = prepare_next_tx_xfer( serial, tx_buff, TX_FIFO_SIZE );

                if( tx_size )
                {
                    /* Write file. */
                    ret = WriteFile( serial->serial, \
                                    tx_buff, \
                                    tx_size, \
                                    NULL, \
                                    &ovTx);
                    if( ! ret && ( ERROR_IO_PENDING != GetLastError() )
                    {
                        hasToContinue = false;
                    }
                }
            }
            else
            {
                if( ERROR_IO_PENDING != GetLastError( ) )
                {
                    hasToContinue = false;
                }
            }
        break;
        
        default:
            /* Error invalid event. */
            hasToContinue = true;
        break;
        }
    }
    return 0;
}

static uint32_t prepare_next_tx_xfer( serial_handle_t * serial,
                                 uint8_t * tx_buff,
                                 uint32_t buff_size )
{
    uint32_t i = 0;

    ASSERT( (NULL == tx_buff ), 0 );
    ASSERT( ( 0 == buff_size ), 0 );

    EnterCriticalSection( &serial->tx_mutex );
    while( ! IS_FIFO_EMPTY( serial->tx ) && ( i < buff_size ) )
    {
        tx_buff[i] = serial->tx->data[serial->tx->ptr_out];
        INC_PTR_OUT( serial->tx );
       
        i++;
    }

    if( 0 == i )
    {
        serial->tx_process = false;
    }
    LeaveCriticalSection( &serial->tx_mutex );

    return i;
}
