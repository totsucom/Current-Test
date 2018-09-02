/*
 * 消費電流を測定するテスト用
 */
#include <string.h>         // C 標準ライブラリ用
#include <AppHardwareApi.h>
#include "utils.h"
#include "ToCoNet.h"
#include "serial.h"         // シリアル用
#include "sprintf.h"        // SPRINTF 用
#include "ToCoNet_mod_prototype.h" // ToCoNet モジュール定義(無線で使う)

//設定オプション
#define SLEEP           FALSE    // スリープするか
#define RAM_OFF         FALSE    // スリープでRAMをOFFするか (1μAほど節約のはず)
#define RX_ENABLE       TRUE     // 受信回路を開くか
#define TX_ENABLE       TRUE     // 連続送信する (テスターでは瞬時電流を測れないのでこれで試す)
#define TX_POWER        0        // 送信出力 3:最大 0:最小
#define ADC_POWER       TRUE     // アナログ部電源ON
#define ADC_MEASURING   FALSE     // ADC変換連続実行

/*
 * 考えられる組み合わせ   0:False 1:True
 *
 *                        SLEEP RAM_OFF RX_ENABLE TX_ENABLE   TX_POWER  ADC_POWER   ADC_MEAS
 * SLEEP                    1      1       -          -          -          -          -
 * SLEEP+RAM                1      0       -          -          -          -          -
 * WAKE                     0      -       0          0          -          -          -
 * WAKE+RX(IDLE)            0      -       1          0          -          -          -
 * WAKE+ADC_STANDBY         0      -       0          0          -          1          0
 * WAKE+ADC_MEASURING       0      -       0          0          -          1          1
 *
 * WAKE+TX3(CONT)           0      -       0          1          3 　　×テスト失敗。テスター電流値がバラついて測れない（低く出る）
 * WAKE+RX(IDLE)+TX0(CONT)  0      -       1          1          0 　　×テスト失敗。同上
 * WAKE+RX(IDLE)+TX1(CONT)  0      -       1          1          1 　　×テスト失敗。同上
 * WAKE+RX(IDLE)+TX2(CONT)  0      -       1          1          2 　　×テスト失敗。同上
 * WAKE+RX(IDLE)+TX3(CONT)  0      -       1          1          3 　　×テスト失敗。同上
 */

#define UART_BAUD 115200 	        // シリアルのボーレート
static tsFILE sSerStream;           // シリアル用ストリーム
static tsSerialPortSetup sSerPort;  // シリアルポートデスクリプタ

// ToCoNet 用パラメータ
#define APP_ID   0x67721122
#define CHANNEL  15

// 送信開始フラグ
bool bStartSend;

// デバッグメッセージ出力用
#define debug(...) vfPrintf(&sSerStream, LB __VA_ARGS__)

// デバッグ出力用に UART を初期化
static void vSerialInit() {
    static uint8 au8SerialTxBuffer[96];
    static uint8 au8SerialRxBuffer[32];

    sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
    sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
    sSerPort.u32BaudRate = UART_BAUD;
    sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
    sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
    sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
    sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
    sSerPort.u8SerialPort = E_AHI_UART_0;
    sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
    SERIAL_vInit(&sSerPort);

    sSerStream.bPutChar = SERIAL_bTxChar;
    sSerStream.u8Device = E_AHI_UART_0;
}

// ハードウェア初期化
static void vInitHardware()
{
	// デバッグ出力用
	vSerialInit();
	ToCoNet_vDebugInit(&sSerStream);
	ToCoNet_vDebugLevel(0);
}

// ブロードキャスト送信を実行
static bool_t sendBroadcast()
{
    tsTxDataApp tsTx;
    memset(&tsTx, 0, sizeof(tsTxDataApp));

    tsTx.u32SrcAddr = ToCoNet_u32GetSerial();//チップのS/N
    tsTx.u32DstAddr = TOCONET_MAC_ADDR_BROADCAST;

    tsTx.bAckReq = FALSE;
    tsTx.u8Retry = 9;       //合計10回連続送信
    tsTx.u8Cmd = TOCONET_PACKET_CMD_APP_DATA;
    tsTx.u16RetryDur = 1;   // 再送間隔[ms]
    tsTx.u16DelayMax = 0;   // 送信開始タイミングはブレなし
    tsTx.u8CbId = 1;
    tsTx.u8Seq = 1;

    // 与えられた文字列を送信
    tsTx.u8Len = 80;
    memcpy(tsTx.auData, "01234567890123456789012345678901234567890123456789012345678901234567890123456789", 80);

    // 送信
    return ToCoNet_bMacTxReq(&tsTx);
}

// ユーザ定義のイベントハンドラ
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg)
{

    switch (pEv->eState) {
    // アイドル状態
    case E_STATE_IDLE:

        if (eEvent == E_EVENT_START_UP) { // 起動時

            if(ADC_POWER){
                // 1) アナログ部の電源投入
                if (!bAHI_APRegulatorEnabled()) {
                    vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE,
                        E_AHI_AP_INT_ENABLE,        // DISABLE にするとアナログ部の電源断
                        E_AHI_AP_SAMPLE_4,          // サンプル数 2,4,6,8 が選択可能
                        E_AHI_AP_CLOCKDIV_500KHZ,   // 周波数 250K/500K/1M/2M
                        E_AHI_AP_INTREF);

                    // レギュレーター部安定待ち
                    while(!bAHI_APRegulatorEnabled()) ;
                }
                if(ADC_MEASURING){
                    // 2) ADC 開始
                    vAHI_AdcEnable(
                            E_AHI_ADC_CONTINUOUS,
                                // E_AHI_ADC_SINGLE_SHOT １回のみ
                                // E_AHI_ADC_CONTINUOUS 連続実行
                            E_AHI_AP_INPUT_RANGE_2,
                                // E_AHI_AP_INPUT_RANGE_1 (0-1.2V)
                                // または E_AHI_AP_INPUT_RANGE_2 (0-2.4V)
                            E_AHI_ADC_SRC_VOLT
                                // E_AHI_ADC_SRC_ADC_1 (ADC1)
                                // E_AHI_ADC_SRC_ADC_2 (ADC2)
                                // E_AHI_ADC_SRC_ADC_3 (ADC3)
                                // E_AHI_ADC_SRC_ADC_4 (ADC4)
                                // E_AHI_ADC_SRC_TEMP (温度)
                                // E_AHI_ADC_SRC_VOLT (電圧)
                            );
                    vAHI_AdcStartSample(); // ADC開始
                }
            }

            if (SLEEP) {
                if(RAM_OFF) {
                    debug("Sleeping...RAM OFF");
                } else {
                    debug("Sleeping...RAM ON");
                }
                ToCoNet_vSleep(
                    E_AHI_WAKE_TIMER_0,
                    3600000,    //1hr
                    TRUE,       //TRUE:インターバル
                    RAM_OFF);   //TRUE:RAMオフ
                WAIT_UART_OUTPUT(E_AHI_UART_0); // UART 出力の完了を待つ
                while(1);

            } else {

                debug("RX_ENABLE: %s", (RX_ENABLE ? "True" : "False"));
                debug("TX_ENABLE: %s", (TX_ENABLE ? "True" : "False"));

                if (TX_ENABLE) {
                    debug("TX_POWER: %d", TX_POWER);

                    debug("Start sending...");
                    bStartSend = TRUE;
                    ToCoNet_Event_SetState(pEv, E_STATE_RUNNING);

                } else {
                    debug("Idling...");
                }
            }
        }
        break;

    // 稼働状態
    case E_STATE_RUNNING:

        if (bStartSend) {
            bStartSend = FALSE;

            // 送信
            sendBroadcast();
        }
        break;

    default:
        break;
    }
}

// メインループ無限ループではなく割り込みなどの発生を起点として呼び出されます
void cbToCoNet_vMain(void)
{
}

// パケット送信完了時
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus)
{
    // 送信開始指示
    bStartSend = TRUE;
}

// パケット受信時
void cbToCoNet_vRxEvent(tsRxDataApp *pRx)
{
}

// 電源オンによるスタート
void cbAppColdStart(bool_t bAfterAhiInit)
{
	if (!bAfterAhiInit) {
        // 必要モジュール登録手続き
        ToCoNet_REG_MOD_ALL();
	} else {
        // 電源電圧が 2.0V まで下降しても稼働を継続
        vAHI_BrownOutConfigure(0, FALSE, FALSE, FALSE, FALSE);

        // SPRINTF 初期化
        SPRINTF_vInit128();

        // ToCoNet パラメータ
        sToCoNet_AppContext.u32AppId = APP_ID;
        sToCoNet_AppContext.u8Channel = CHANNEL;
        sToCoNet_AppContext.bRxOnIdle = RX_ENABLE;  //受信回路
        sToCoNet_AppContext.u8TxPower = TX_POWER;   //送信出力

        // ユーザ定義のイベントハンドラを登録
        ToCoNet_Event_Register_State_Machine(vProcessEvCore);

        // ハードウェア初期化
        vInitHardware();

        if(RX_ENABLE || TX_ENABLE) {
            // MAC 層開始
            ToCoNet_vMacStart();    //無線機能を使わない場合はこれを呼ばない
        }
	}
}

// スリープからの復帰
void cbAppWarmStart(bool_t bAfterAhiInit)
{
}

// ネットワークイベント発生時
void cbToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg)
{
}


// ハードウェア割り込み発生後（遅延呼び出し）
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
}

// ハードウェア割り込み発生時
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
	return FALSE;
}
