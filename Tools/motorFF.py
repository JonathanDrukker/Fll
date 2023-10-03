import statsmodels.api as sm
import numpy as np

print('Starting...')

pathToFile = 'Tools/Data/Logs/rightAnalysis.log'

with open(pathToFile, 'r') as f:
    data = eval(f.read())

x1 = np.array(data['velocity'])
x2 = np.array(data['acceleration'])

x = np.array((x1, x2)).T
y = np.array(data['duty'])

model = sm.OLS(y, x)
Kv, Ka = model.fit().params

Ks = max(i for i, x in enumerate(data['velocity']) if x == 0)

Kv = float("%.5g" % Kv)
Ka = float("%.5g" % Ka)

print(Ks, Kv, Ka, max(x1))
print('Done!')
