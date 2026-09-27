/**
 * @file vofa.h
 * @brief VOFA+ 涓婁綅鏈烘暟鎹彂閫?鈥斺€?FireWater 娉㈠舰鍗忚鎺ュ彛
 * @note  娉㈠舰鍗忚涓?FireWater(ASCII 鏂囨湰)锛屽抚鏍煎紡: ch0,ch1,ch2\r\n
 *        渚? -1.2345,0.0000,12.5000\r\n
 *        鍦?VOFA+ 涓渶閫夋嫨 "FireWater" 鍗忚锛屽苟鎸夐€楀彿椤哄簭缁戝畾閫氶亾銆? */
#ifndef __VOFA_H
#define __VOFA_H

#include "stm32f4xx_hal.h"

/** @brief 灏忔暟浣嶆暟锛堝畾鐐规樉绀猴紝绛変环浜庡師 printf 鐨?%.4f锛?*/
#define VOFA_FRAC_DIGITS 4
/** @brief 娴偣鏀惧ぇ鍊嶆暟锛屽繀椤讳负 10^VOFA_FRAC_DIGITS */
#define VOFA_FRAC_SCALE 10000u
/**
 * @brief 杈撳嚭闄愬箙锛堢粷瀵瑰€硷級銆傝秴鍑鸿閲忕▼锛堝惈 卤Inf锛夋寜楗卞拰鍊艰緭鍑猴紝淇濊瘉甯ч暱鏈夌晫銆? * @note  鎹㈢畻缁撴灉涓?uint32 鐨勬斁澶ф暣鏁帮紝闇€婊¤冻 ABS_MAX * VOFA_FRAC_SCALE < 2^32锛? *        褰撳墠 200000 * 10000 = 2.0e9锛岀暀鏈変竴鍊嶄綑閲忥紱鍐嶆斁澶ч噺绋嬮渶鍚屾鏀规鍊笺€? */
#define VOFA_ABS_MAX 200000.0f
/** @brief 鍗曞抚瀛楃缂撳啿闀垮害锛? 閫氶亾鏈€鍧?38 瀛楄妭锛堝惈 \r\n锛夛紝鍙?64 鐣欎綑閲?*/
#define VOFA_LINE_MAX_LEN 64
/**
 * @brief 鍗曞抚鍙戦€佽秴鏃?ms)銆備竴甯ф渶鍧?38 瀛楄妭 @115200 鈮?3.3ms锛?0ms 宸插惈 6 鍊嶄綑閲忋€? * @note  鏁村抚鍙皟鐢ㄤ竴娆?HAL_UART_Transmit锛堟棫瀹炵幇鏄瘡涓瓧绗﹁皟涓€娆°€佸悇甯?31ms
 *        瓒呮椂锛屾渶鍧忓彲绱姞鍒?1 绉掔骇锛夛紱涓斿彂閫佸墠宸茬敤 gState 鍒ゅ繖锛屽繖鏃剁洿鎺ヤ涪甯э紝
 *        鎵€浠ユ甯告儏鍐典笉浼氳蛋鍒拌繖涓秴鏃躲€? */
#define VOFA_TX_TIMEOUT_MS 20

/**
 * @brief  鍒濆鍖?VOFA 涓插彛鎺ユ敹锛堝彂閫佹棤闇€鍒濆鍖栵紝澶嶇敤 huart2锛? */
void vofa_init(void);

/**
 * @brief  鎸?FireWater 鍗忚鍙戦€?3 涓诞鐐归€氶亾锛屾湯灏捐嚜鍔ㄨˉ \r\n
 * @param  x1,x2,x3 涓変釜閫氶亾鐨勬诞鐐瑰€? * @note   浠呴檺浠诲姟涓婁笅鏂囪皟鐢紱鑻ヤ笂涓€娆″彂閫佸皻鏈畬鎴愬垯鏈抚鐩存帴涓㈠純锛? *         涓嶄細鍦ㄦ帶鍒朵换鍔￠噷鎺掗槦绛夊緟锛屼篃涓嶄細閫掑綊/鍘嬫爤杩囨繁銆? */
void vofa_send(float x1, float x2, float x3);

#endif

