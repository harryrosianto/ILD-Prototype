import sounddevice as sd
import librosa
import numpy as np
from tensorflow.keras.models import load_model
from sklearn.preprocessing import LabelEncoder
import time
import os
from azure.storage.blob import BlockBlobService
import pandas
from datetime import datetime
from scipy.io.wavfile import write

start_time = time.time()
fs = 22050
duration = 4
path_recording = "hasilprediksi.wav"

recording = sd.rec(duration * fs, samplerate=fs, channels=1, dtype='float64').ravel()
sd.wait()
write(path_recording, fs, recording)
print("Capture Complete")

recsound = {'data': recording, 'rate': 22050}
recsound_mfccs = librosa.feature.mfcc(y=recsound['data'], sr=recsound['rate'], n_mfcc=40)

model = load_model('./saved_models/model_cnn.hdf5')
le = LabelEncoder()
le.classes_ = np.load('./saved_models/classes.npy')

num_rows = 40
num_columns = 174
num_channels = 1
max_pad_len = 174

sample = recsound

mfccs = librosa.feature.mfcc(y=sample['data'], sr=sample['rate'], n_mfcc=40)
pad_width = max_pad_len - mfccs.shape[1]
mfccs = np.pad(mfccs, pad_width=((0, 0), (0, pad_width)), mode='constant')

prediction_feature = mfccs
prediction_feature = prediction_feature.reshape(
        1, num_rows, num_columns, num_channels)

predicted_vector = np.argmax(model.predict(prediction_feature), axis=-1)
predicted_class = le.inverse_transform(predicted_vector)
print("The predicted class is:", predicted_class[0], '\n')

predicted_proba_vector = model.predict(prediction_feature)
predicted_proba = predicted_proba_vector[0]
for i in range(len(predicted_proba)):
    category = le.inverse_transform(np.array([i]))
    print(category[0], "\t\t : ", format(predicted_proba[i], '.32f'))

data = pandas.read_csv('./dataprediksi/hasil_prediksi.csv')
kolom= int(len(data.columns))

now = datetime.now()
current_time = now.strftime("%H:%M:%S")
data.insert(kolom - (kolom-1), current_time, predicted_proba)

data.to_csv('hasil_prediksi.csv', index=False)
print("CSV file created")

root_path = '.'
dir_name = 'dataprediksi'
path = f"{root_path}/{dir_name}"
file_names = os.listdir(path)

account_name = 'hartastorage'
account_key = 'kAhYlJ/javru6QU/j/1/Yy3cxod5VktVsiEDIhAVA574/9ggK+LrrnVa4k/NWV+TxTk79YO72qRYy0y1cWfHHw=='
container_name = 'dataprediksi'

block_blob_service = BlockBlobService(
    account_name=account_name,
    account_key=account_key
)

for file_name in file_names:
    blob_name = f"{dir_name}/{file_name}"
    file_path = f"{path}/{file_name}"
    block_blob_service.create_blob_from_path(container_name, blob_name, file_path)

print("CSV file uploaded")
print("--- %s seconds ---" % (time.time() - start_time))