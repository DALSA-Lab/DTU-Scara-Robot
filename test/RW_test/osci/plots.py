import matplotlib.pyplot as plt
import pandas as pd


# Load data
df_100 = pd.read_csv('test/RW_test/osci/100kHz.CSV')
df_400 = pd.read_csv('test/RW_test/osci/400kHz.CSV')

# Plot
plt.figure(figsize=(6.8, 4.2))
x = range(len(df_100['SDA']))
# plt.plot(data_CM.iloc[:, 18]) # j1 position
plt.plot(df_100['SDA'], label='SDA')
plt.plot(df_100['SCL'], label='SCL')

# plt.xticks(x, df_100['SDA'])
# plt.xlabel('Month')
# plt.ylabel('Sales')
plt.legend()
# plt.show()


# Plot
plt.figure(figsize=(6.8, 4.2))
x = range(len(df_400['SDA']))
# plt.plot(data_CM.iloc[:, 18]) # j1 position
plt.plot(df_400['SDA'], label='SDA')
plt.plot(df_400['SCL'], label='SCL')

# plt.xticks(x, df_100['SDA'])
# plt.xlabel('Month')
# plt.ylabel('Sales')
plt.legend()
plt.show()

