
from roboflow import Roboflow
rf = Roboflow(api_key="OxsE6hovH5cWzwoBAiMA")
project = rf.workspace("astro-wzc1p").project("apple-dataset-qdf9c-cn3zw")
version = project.version(3)
dataset = version.download("yolov8")
                
                

print("다운로드 완료:", dataset.location)
import os
print(os.listdir(dataset.location))