# Camera Based 2D Feature Tracking 
### MP.1 Data Buffer Optimization
I have created a ring buffer with two elements. It checks if the size of the vector is equal to or greater than two and if it is affirmative, it eliminates the first element of the vector. The next element is added to the end of the vector

### MP.2 Keypoint Detection
The *HarrisCorner* detector has been developed in the *detKeypointsHarris* function of the *matching2D_Student* file. The opencv function *cornerHarris* has been called, then it has been normalized and finally the *NMS* method, learned in previous lessons, has been applied.
The rest of detectors *(FAST, BRISK, ORB, AKAZE and SIFT)* have been developed in the *detKeypointsModern* function using the OpenCV libraries.

### MP.3  Keypoint Removal
The  keypoints outside of the proposed rectangle have been removed. To do this, all  elements of the vector *keypoints* have been checked and those whose coordinates are outside of the proposed ROI have been removed.

### MP.4  Keypoint Descriptors
The descriptors *BRIEF, ORB, FREAK, AKAZE* and *SIFT* have been developed in the function *descKeypoints* of the file *matching2D_Student* using the libraries of OpenCV

### MP.5  Descriptor Matching
*FLANN* matching and *k-nearest neighbor* selection have been implemented in the *matchDescriptors* function of the file *matching2D_Student*. The steps performed in the previous lessons have been followed.

### MP.6  Descriptor Distance Ratio
A distance descriptor ratio has been implemented with the guidelines of the previous lessons. The established ratio threshold is 0.8

### MP.7 Performance Evaluation 1
| Detector | Keypoints number | Distribution
| ------ | ------ | ------ |
| ShiTomasi | 1156 | Detects keypoints in all areas of the vehicle (roof, license plate, rear window and body). The distribution is regular, the keypoints are not located excessively close
| HarrisCorner | 1092 | The distribution is not regular. Only kps are detected in the roof and some in the rear window and in the car headlights. Also the kps are very close
| FAST | 3966 | It detects many keypoints and in all areas of the vehicle. I think that because of the number of detected points, the separation reached in  keypoints is correct. Good distribution
| BRISK | 2711 | The distribution is very similar to FAST method distribution. I think the distribution achieved by FAST is better, but this may be due to the large number of key points detected
| ORB | 1112 | In general, detection is poor. Few points are detected and only on the roof and  vehicle headlights
| AKATE | 1655 | Detection and distribution are good. Although it does not detect points in the license plate. I think license plate detection can be a very important point. Behavior is better than ORB and worse than BRISK and FAST
| SIFT | 1371 | The distribution is the most correct for the number of points detected  (less than FAST, BRISK and AKAZE). The keypoints are in all areas of the vehicle. I think the point separation area is the best of all algorithms

### MP.8 Performance Evaluation 2
The results of this task can be found in the Table.xlsx file sent with the project.

### MP.9 Performance Evaluation 3
| TOP Level | Detector | Descriptor | Justification
| ------ | ------ | ------ | ------ |
| 1 | FAST | BRISK | It is the fastest combination (detector + descriptor). It also has the highest number of matched keypoints. Also the detector distribution is correct |
| 2 | AKAZE | BRISK | El numero de matched keypoints es alto. El descriptor es muy rápido y el comportamiento del detector es bueno  |
| 3 | SIFT | SIFT | I think SIFT detector is one of the best, although the combination with its descriptor is not the best of the existing ones. Processing times are longer than in the previous options and the percentage of matched keypoints is lower. The detector distribution is very good |
