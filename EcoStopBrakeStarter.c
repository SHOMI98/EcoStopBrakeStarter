//! @file EcoStopBrakeStarter.c
//! @brief EcoStopBarekeStarter for ATTiny13A
//! @author SHOMI98
//!
//! Copyright 2019 SHOMI98
//!
//! Licensed under the Apache License, Version 2.0 (the "License");
//! you may not use this file except in compliance with the License.
//! You may obtain a copy of the License at
//!
//!     http://www.apache.org/licenses/LICENSE-2.0
//!
//! Unless required by applicable law or agreed to in writing, software
//! distributed under the License is distributed on an "AS IS" BASIS,
//! WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//! See the License for the specific language governing permissions and
//! limitations under the License.

#define F_CPU    128000UL				//!< CPU Clock:128kHz

#include <util/delay.h>
#include <avr/io.h>
#include <stdbool.h>

//! @name 時間設定
//@{
#define POLLING_INTERVAL		100		//!< ポーリング間隔(ms)
#define CRANKING_MAX_TIME		4000	//!< エンジン始動待ち最大時間(ms)
#define START_CHECK_DELAY_TIME	500		//!< エンジン始動判定開始遅延間隔(ms)
#define START_THRESHOLD_TIME	500		//!< エンジン始動完了とするL端子信号検出時間(ms)
//@}

//! @name 入力端子
//@{
#define IN_DISABLE	_BV(PINB0)	//!< アイドリングストップ有効・無効入力
#define IN_N		_BV(PINB1)	//!< N信号入力
#define IN_BRAKE	_BV(PINB2)	//!< ブレーキ入力
#define IN_L		_BV(PINB5)	//!< L端子入力
//@}

//! @name 出力端子
//@{
#define OUT_BRAKE	_BV(PORTB3)	//!< ブレーキ、スターターカットリレー出力
#define OUT_N		_BV(PORTB4)	//!< EcoStop用N信号出力
//@}

#define IO_DIR (_BV(DDB3) | _BV(DDB4))    //!< 入出力方向:PB0～2を入力、PB3～PB4を出力に設定

#define INPUT_PULL_UP (_BV(PORTB0) | _BV(PORTB1) | _BV(PORTB2))	//!< プルアップ設定:PB0～2をプルアップ

//! @brief アイドリングストップ有効・無効状態
//! @return true:アイドリングストップ無効
//! @return false:アイドリングストップ有効
bool isDisabled()
{
	// 有効時L
	return (PINB & IN_DISABLE);
}

//! @brief ニュートラル状態
//! @return true:ニュートラル
//! @return false:ニュートラル以外
bool isNeutral()
{
	// ニュートラル時H
	return (PINB & IN_N);
}

//! @brief ブレーキ解除状態
//! @return true:ブレーキ解除
//! @return false:ブレーキ中
bool isBrakeReleased()
{
	// ブレーキ解除でH
	return (PINB & IN_BRAKE);
}

//! @brief エンジン動作状態
//! @return true:動作中
//! @return false:停止中
bool isStarted()
{
	// エンジン動作中L
	return !(PINB & IN_L);
}

//! @brief シフト位置伝達
//! @note アイドリングストップ無効時はニュートラルとして伝達する
void transferShiftPosition()
{
	if(isDisabled() || isNeutral())
	{
		// ニュートラルにする
		PORTB &= ~OUT_N;
	}
	else
	{
		// ニュートラル以外
		PORTB |= OUT_N;
	}	
}

//! @brief クランキング
//! 最大でCRANKING_MAX_TIME(ms)の間ブレーキ、スターターカットリレー接続し、ニュートラルにされたふりをしてEco-Stopにクランキングさせる
//! START_CHECK_DELAY_TIME(ms)経過後、START_THRESHOLD_TIME(ms)以上の間エンジン始動状態と判定された場合は始動完了とする
//! @attention クランキングはEco-Stopが行うため、始動完了と判定してスターターカットリレー切断した後も
//! Eco-Stopがクランキングをやめるまでの間は実際のシフトレバーをニュートラルにするとセルが回ってしまいます
void cranking(void)
{
	// ブレーキ、スターターカットリレー接続し、ニュートラルにされたふりをしてEco-Stopにクランキングさせる
	PORTB = (PORTB | OUT_BRAKE) & ~OUT_N;

	// 最大でCRANKING_MAX_TIME(ms)の間始動待ち
	int start_time = 0;
	for(int cranking_time = 0; cranking_time < CRANKING_MAX_TIME;)
	{
		_delay_ms(POLLING_INTERVAL);
		cranking_time += POLLING_INTERVAL;

		if(START_CHECK_DELAY_TIME >= cranking_time
		|| !isStarted())
		{
			// 始動判定開始時間前もしくは始動できていなければ、L端子検出開始時間クリア
			start_time = 0;
			continue;
		}

		if(!start_time)
		{
			// L端子信号開始時間記憶
			start_time = cranking_time;
		}
		
		if(START_THRESHOLD_TIME <= cranking_time - start_time)
		{
			// L端子信号検出期間がしきい値を超えていれば始動完了
			break;	
		}
	}

	// ブレーキ、スターターカットリレー切断
	PORTB &= ~OUT_BRAKE;
}

//! @brief 初期化処理
void init(void)
{
	// コンパレータ無効化
	ACSR |= _BV(ACD);
	// 入出力方向設定
	DDRB = IO_DIR;
	// プルアップ設定
	PORTB = INPUT_PULL_UP;
	// シフト位置反映
	transferShiftPosition();
}

//! @brief メイン処理
int main(void)
{
	// 初期化
	init();

	// 手動始動済みフラグクリア
	bool isManualStarted = false;
	
	// 前回入力状態
	unsigned char lastInput = PINB;
	while(1)
	{
		_delay_ms(POLLING_INTERVAL);

		// 手動始動済みフラグ更新
		bool started = isStarted();
		isManualStarted |= started;

		// 手動始動後のエンジン停止中かつ
		// ・今回ブレーキを解除した
		// ・アイドリングストップ無効
		// いずれかの場合には始動を試みる
		unsigned char input = PINB;
		if(isManualStarted
		&& !started
		&& ((((lastInput ^ input) & IN_BRAKE)
				&& isBrakeReleased())
			|| isDisabled()))
		{
			cranking();
			
			if(!isStarted())
			{
				// 始動出来なかった場合はエンジン始動されるまで再度のクランキング処理はしない
				isManualStarted  = false;
			}
		}
		// 前回入力状態更新
		lastInput = input;

		// シフト位置反映
		transferShiftPosition();
	}
	return 0;
}
