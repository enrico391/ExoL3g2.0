import numpy as np
import pandas as pd
from microlearn.offloader import Offload
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC # Or you can use LinearSVC
# Parsing the dataset
dataset = pd.read_csv('datiSingleMPU_Adafruit.txt', sep=',', header=0)
X_train = dataset.loc[:, 'Ax':'Az'].to_numpy()
Y_train = dataset.loc[:, 'Occupancy'].to_numpy()
# Training the StandardScaler
ss = StandardScaler()
X_train_ss = ss.fit_transform(X_train)
# Training the ML model
linsvm = SVC(C=0.1, kernel='linear') # Or LinearSVC(C=0.1)
linsvm.fit(X_train_ss, Y_train)
# Offloading the trained model to Arduino code
off = Offload(linsvm, ss)
off.export_to_arduino('linsvm_offload.ino')