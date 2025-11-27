import matplotlib.pyplot as plt
import pandas as pd
import sys
plt.style.use('../thesis_style.mplstyle')

if len(sys.argv[1:]) != 1:
    print("Specify a file name")
    quit()

# Load data
base_file = ".".join(sys.argv[1].split(".")[0:-1])
df = pd.read_csv(sys.argv[1],skipinitialspace=True)


# Plot
# plt.figure(figsize=(11,11))
plt.hist(df['read [us]'], label=f'read (mean: {df['read [us]'].mean():.2f} max: {df['read [us]'].max():.2f})')
plt.hist(df['write [us]'], label=f'write (mean: {df['write [us]'].mean():.2f} max: {df['write [us]'].max():.2f})')
# plt.text(0, -1, r'Hello, world!', fontsize=15)

plt.ylabel(r'Samples')
plt.xlabel(r'$t_{total}$ [$\mu $s]')
plt.legend()


plt.tight_layout()
plt.savefig(f"{base_file}.pdf")
plt.savefig(f"{base_file}.png")
plt.show()
