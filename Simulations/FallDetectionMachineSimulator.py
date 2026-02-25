import numpy as np
import matplotlib.pyplot as plt

# config
G_TRIGGER = 2.2
SAMPLE_RATE_MS = 20
ITERATIONS = 10000

# Ported over from wokwi
def run_wokwi_logic_sim(sensor_data, return_states=False):
    in_free_fall, waiting_impact, fall_detected = False, False, False
    ff_start, impact_deadline = 0, 0
    states = [] # 0:idle, 1:freefall, 2:impact, 3:falldetected

    for t, total_g in enumerate(sensor_data):
        now = t * SAMPLE_RATE_MS
        
        # state transition
        if not in_free_fall and not waiting_impact and total_g < 0.5:
            in_free_fall, ff_start = True, now
        elif in_free_fall and total_g >= 0.5:
            if (now - ff_start) >= 60:
                waiting_impact, impact_deadline = True, now + 400
            in_free_fall = False
        elif waiting_impact:
            if total_g > G_TRIGGER:
                fall_detected, waiting_impact = True, False
            elif now > impact_deadline:
                waiting_impact = False

        state = 3 if fall_detected else (2 if waiting_impact else (1 if in_free_fall else 0))
        states.append(state)
                
    return (fall_detected, states) if return_states else fall_detected

# run testcases
def run_test_suite():
    fp = sum(1
             for _ in range(ITERATIONS) if run_wokwi_logic_sim(np.random.normal(1.0, 0.4, 500))
             )
    
    tp = 0
    for _ in range(ITERATIONS):
        fall_data = np.random.normal(1.0, 0.05, 150)
        
        fall_data[50:55], fall_data[55:58] = 0.1, 4.0
        
        if run_wokwi_logic_sim(fall_data): tp += 1
    return fp, tp

fp_count, tp_count = run_test_suite()

# --- Visualization ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# 1. Fall Sample Plot
sample_data = np.random.normal(1.0, 0.05, 150)
sample_data[50:55], sample_data[55:58] = 0.1, 3.5
detected, states = run_wokwi_logic_sim(sample_data, return_states=True)
time = np.arange(len(sample_data)) * SAMPLE_RATE_MS

ax1.plot(time, sample_data, color='#003f5c', lw=1.5, label='Logged G force')
ax1.axhline(0.5, color='#ffa600', ls='--', label='Freefall threshold')
ax1.axhline(G_TRIGGER, color='#ff6361', ls='--', label='Impact threshold')

# Highlight active states
for i in range(1, len(states)):
    if states[i] == 1: ax1.axvspan((i-1)*20, i*20, color='#ffa600', alpha=0.3)
    if states[i] == 2: ax1.axvspan((i-1)*20, i*20, color='#ff6361', alpha=0.2)

if detected:
    idx = states.index(3)
    ax1.annotate('Fall detected', xy=(idx*20, sample_data[idx]), xytext=(idx*20+100, 4),
                 arrowprops=dict(arrowstyle='->', lw=2), weight='bold', color='green')

ax1.set_title("Fall detection Machine Analysis", fontweight='bold')
ax1.set_ylabel("G-Force")
ax1.legend(loc='upper right', frameon=True)

# 2. Performance Bar Chart
labels = ['False positives', 'True positives']
rates = [fp_count/ITERATIONS*100, tp_count/ITERATIONS*100]
bars = ax2.bar(labels, rates, color=['#ff6361', '#58508d'], width=0.4)

ax2.set_title(f"False positive reports (n={ITERATIONS})", fontweight='bold')
ax2.set_ylabel("Success rate (%)")
ax2.set_ylim(0, 110)

for bar in bars:
    ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 2, 
             f'{bar.get_height():.2f}%', ha='center', weight='bold')

plt.tight_layout()
plt.show()

print(f"FP: {fp_count} | TP: {tp_count}")
