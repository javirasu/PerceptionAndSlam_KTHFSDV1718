# Perception

Refer to [Installation](Installation.md) for instructions on installing Tensorflow object detection api.

Presently, we use the SSD mobilenet v1 from tensorflow models directory. This turnkey algorithm was chosen mainly because of ease of deployment. The following section covers steps to train the model. We then refer to our recent progress in making slimmer SSDs.    

# Training the model
Try to run “object_detection_tutorial.ipynb” in models/research/object_detection/

Convert the above .ipynb file to .py file and try to run it for your webcam using opecv cv2 library.

## Custom Object Detection
Root directory : traffic_cone_detection 

- First, create a folder named “traffic_cone_detection" in desktop. Create a subfolder named images. Create two subsub folders named train and test.  

- Create dataset of images and divide it into train and test folder
 
- Start annotating images. This will convert images to .xml file. Write a code segment to create training data (TODO: Javier's traning data sampling script)

- Convert .xml files in training and test set to .csv file using function “xml_to_csv.py” (NOTE that csv files are being stored in a folder named “data” so if that folder doesnt exist, create a folder of that name in root directory)

- Convert .csv file to TF Records using function “generate_tfrecords.py”. The call looks like this
python generate_tfrecord.py --csv_input=data/train_labels.csv  --output_path=data/train.record

- Run setup.py in models/research

- Download model and config files. You can find a list of [all models](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md) and a list of all available [config files](https://github.com/tensorflow/models/tree/master/research/object_detection/samples/configs). 

Now modify the config file : change everything with “PATH_TO_BE_CONFIGURED”
- line 8, change number of classes to 1 instead of 37
- line 156, finetune check point, change to “ssd_mobilenet_v1_coco_2017_11_17/model.ckpt”
- line 175, input path: "data/train.record"
- line 177, label_map_path: "training/traffic_cone_detection.pbtxt"
(create a folder called “training” and create a document called “traffic_cone_detection.pbtxt”. NOTE: also create a copy of this pbtxt file in data folder)
- line 189, input path: "data/test.record"
- line 191, label_map_path: same as line 177 

Now add following code to traffic_cone_detection.pbtxt:

`item {
    id: 1
    name: 'Cone'    
}`

Move following files/folders from traffic_cone_detection to models/research/object_detection:
- data folder
- images folder
- training folder
- ssd mobilenet unzipped model folder
- ssd mobilenet config file

Move the config file in training folder

Ensure all files are in correct place. Call predict using following command: 

`python train.py --logtostderr --train_dir=training/ --pipeline_config_path=training/ssd_mobilenet_v1_pets.config`

Source: 
- [Sentdex object detection](https://www.youtube.com/playlist?list=PLQVvvaa0QuDcNK5GeCQnxYnSSaar2tpku)
- [Step-by-step Tensorflow Object Detection API](https://medium.com/@WuStangDan/step-by-step-tensorflow-object-detection-api-tutorial-part-1-selecting-a-model-a02b6aabe39e)

## GPU Options 
Using the gpu options, you can constrain the gpu utilization of tensorflow in runtime (and training time as well maybe? But why would you do that). Refer to [this](https://devblogs.nvidia.com/tensorrt-integration-speeds-tensorflow-inference/) link for more details. For SSD mobilenet v1, were able to restrict gpu usage to ~2.4 Gb. 

## Slimmer SSDs
It's prudent to keep pushing the runtime GPU utilization for deep learning models, as it'll enable us to implement multiple algorithms (and by extension use more cameras) for robustness of perception. We made some progress in this direction through our Deep learning project course. Refer to [this](https://github.com/ajinkyakhoche/Object-Detection-Project) repo for more details 
