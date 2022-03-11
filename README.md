# ILD-Prototype
a Final Project for Illegal Logging Detection using Raspberry Pi 4 Model B with MFCC and CNN method

![](scientific_poster.jpg)

### Prerequisites

Function dependencies used in this project:

- Keras 2.4.3
- matplotlib 3.2.2
- numpy 1.19.5
- pandas 1.1.5
- tensorflow 2.3.0
- librosa 0.8.0
- azure-storage-blob==2.1.0

### Classes
At this time, only 5 classes will be picked for the Illegal Logging Detection using Raspberry Pi
`suara_gergaji suara_tembakan angin suara_alam orang_berbicara`

### Audio Processing
Data generation involves producing raw PCM wavform data containing a desired number of samples and at fixed sample rate and the following configuration is used

| Samples        | Sample Rate           | Clip Duration (ms)  |
| ------------- |:-------------:| -----:|
| 22050      | 22050 | 4000 |

### Built With

* [Keras](https://keras.io/) - Deep Learning Framework
* [TensorFlow](http://tensorflow.org/) - Machine Learning Library

### Results
After the training the model for 75 epochs, the following confusion matrix was generated for assessing classification performance.

![](matrix.jpg)
