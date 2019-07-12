/*
quu..__
 $$$b  `---.__
	"$$b        `--.                          ___.---uuudP
	 `$$b           `.__.------.__     __.---'      $$$$"              .
		 "$b          -'            `-.-'            $$$"              .'|
			 ".                                       d$"             _.'  |
				 `.   /                              ..."             .'     |
					 `./                           ..::-'            _.'       |
						/                         .:::-'            .-'         .'
					 :                          ::''\          _.'            |
					.' .-.             .-.           `.      .'               |
					: /'$$|           .@"$\           `.   .'              _.-'
				 .'|$u$$|          |$$,$$|           |  <            _.-'
				 | `:$$:'          :$$$$$:           `.  `.       .-'
				 :                  `"--'             |    `-.     \
				:##.       ==             .###.       `.      `.    `\
				|##:                      :###:        |        >     >
				|#'     `..'`..'          `###'        x:      /     /
				 \                                   xXX|     /    ./
					\                                xXXX'|    /   ./
					/`-.                                  `.  /   /
				 :    `-  ...........,                   | /  .'
				 |         ``:::::::'       .            |<    `.
				 |             ```          |           x| \ `.:``.
				 |                         .'    /'   xXX|  `:`M`M':.
				 |    |                    ;    /:' xXXX'|  -'MMMMM:'
				 `.  .'                   :    /:'       |-'MMMM.-'
					|  |                   .'   /'        .'MMM.-'
					`'`'                   :  ,'          |MMM<
						|                     `'            |tbap\
						 \                                  :MM.-'
							\                 |              .''
							 \.               `.            /
								/     .:::::::.. :           /
							 |     .:::::::::::`.         /
							 |   .:::------------\       /
							/   .''               >::'  /
							`',:                 :    .'
																	 `:.:'
 
*/
#include "FreeRTOS.h"
#include "task.h"
#include "detect_task.h"

#define errorListLength 10
extern void send_judge_info(uint16_t send_id,uint16_t kehuduan_id,uint16_t num,const float data);
/*  */
/*  */
static error_t errorList[errorListLength+1];
//掉线判断任务
void DetectTask(void *pvParameters)
{
	static uint32_t time_add_;
	//空闲一段时间
	vTaskDelay(1800);
	while (1)
	{
		time_add_++;
		if(time_add_%50==0)
		{
			
		}
		for(uint8_t i=0;i<errorListLength;i++)
		{
			errorList[i].Losttime=xTaskGetTickCount()-errorList[i].regTime;
			if(errorList[i].Losttime>errorList[i].overtime)
			{
				errorList[i].lost_flag=1;
			}
			else
			{
				errorList[i].lost_flag=0;
			}
		}
		vTaskDelay(2);
	}
}
//设备接收数据钩子函数
void DetectHook(uint8_t toe)
{
	errorList[toe].regTime= xTaskGetTickCount();
}
void DetectInit(uint16_t i,uint32_t over_time)//over_time超时时间
{
		errorList[i].regTime  = xTaskGetTickCount();
		errorList[i].overtime = over_time;
}
//判断是否掉线，返回1掉线
uint8_t Detect_Judeg(uint8_t toe)
{
	if(errorList[toe].lost_flag==1)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
