"""
python3 fft.py (csvファイル 複数可)
サンプリング周波数: 1000Hz
電極は P3, P4, O1, O2, Pz ppの6つ
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys


df = pd.read_csv('raw_imu.csv', encoding='shift-jis', skiprows=6, usecols=[6])
# fft計算

F = np.fft.fft(df, axis=0)

Amp = np.abs(F)

data_length = len(df)
# プロット用データ dataの長さを1000等分する。
freq = np.linspace(0, 1000, data_length)

print(np.nonzero(Amp>4700)[0])
# グラフ表示
plt.figure()
plt.rcParams['figure.subplot.bottom'] = 0.2
plt.plot(freq, Amp)
plt.xticks(np.arange(-100,1100,20))
plt.xlabel('Frequency [Hz]', fontsize=20)
plt.ylabel('Amplitude [m/s]', fontsize=20)
plt.xlim(0, 1000)
plt.ylim(0, 8000)
plt.grid()
plt.show()
