
#include "iwdg.h"

void Iwdg::init()
{
	/* USER CODE END IWDG_Init 1 */
	LL_IWDG_Enable(IWDG);
	LL_IWDG_EnableWriteAccess(IWDG);
	LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_4);
	LL_IWDG_SetReloadCounter(IWDG, 4095);
	while (LL_IWDG_IsReady(IWDG) != 1)
	{
	}

	LL_IWDG_ReloadCounter(IWDG);
}

void Iwdg::feed()
{
	LL_IWDG_ReloadCounter(IWDG);
}
Iwdg iwdg;

