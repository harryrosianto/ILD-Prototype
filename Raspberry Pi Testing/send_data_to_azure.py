import sounddevice as sd
import librosa
import numpy as np
from keras.models import load_model
from sklearn.preprocessing import LabelEncoder

fs = 22050
duration = 4

recording = sd.rec(duration * fs, samplerate=fs, channels=1, dtype='float64').ravel()
sd.wait()
print("Capture Complete")

recsound = {'data': recording, 'rate': 22050}
recsound_mfccs = librosa.feature.mfcc(y=recsound['data'], sr=recsound['rate'], n_mfcc=40)

model = load_model('./saved_models/weights.best.basic_cnn.hdf5')
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