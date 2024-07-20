# import roboflow

# rf = roboflow.Roboflow(api_key="8H624Am5CFU0bbi8aNQn")

# project = rf.workspace().project("gazebo_mpc")
# model = project.version("1").model

# # optionally, change the confidence and overlap thresholds
# # values are percentages
# model.confidence = 50
# model.overlap = 25

# # predict on a local image
# prediction = model.predict("./drone_frames/frame_25.jpg")
# print (prediction)
# # Predict on a hosted image via file name
# # prediction = model.predict("YOUR_IMAGE.jpg", hosted=True)

# # # Predict on a hosted image via URL
# # prediction = model.predict("https://...", hosted=True)

# # Plot the prediction in an interactive environment
# prediction.plot()

# # Convert predictions to JSON
# prediction.json()

from inference import get_model
import cv2
model = get_model(model_id="gazebo_mpc/1",api_key="8H624Am5CFU0bbi8aNQn")

image = cv2.imread("./drone_frames/frame_25.jpg")
results = model.infer(image)[0]
print (results.predictions[0].x)