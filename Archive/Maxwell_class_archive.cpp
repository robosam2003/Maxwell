//
// Created by SamScott on 02/03/2026.
//
void Maxwell::sync_timer_frequencies(long pwm_frequency) {
    auto timers = {pwm_3x->TIM_A, pwm_3x->TIM_B, pwm_3x->TIM_C};
    uint32_t min_freq = -1;
    uint32_t max_freq = 0;
    for (auto timer : {pwm_3x->TIM_A, pwm_3x->TIM_B, pwm_3x->TIM_C}) {
        uint32_t freq = timer->getTimerClkFreq();
        if (freq > max_freq) {
            max_freq = freq;
        }
        if (freq < min_freq) {
            min_freq = freq;
        }
    }
    uint32_t overflow = min_freq / pwm_frequency;


    for (auto timer : {pwm_3x->TIM_A, pwm_3x->TIM_B, pwm_3x->TIM_C}) {
        timer->setPrescaleFactor(timer->getTimerClkFreq()/min_freq);
        timer->setOverflow(overflow, TICK_FORMAT);
        timer->refresh();
    }
}