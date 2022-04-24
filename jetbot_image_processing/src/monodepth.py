import cv2
import torch
#import urllib.request
import time
import matplotlib.pyplot as plt

#model_type = "DPT_Large"     # MiDaS v3 - Large     (highest accuracy, slowest inference speed)
#model_type = "DPT_Hybrid"   # MiDaS v3 - Hybrid    (medium accuracy, medium inference speed)
model_type = "MiDaS_small"  # MiDaS v2.1 - Small   (lowest accuracy, highest inference speed)

midas = torch.hub.load("intel-isl/MiDaS", model_type)
torch.save(midas.state_dict(), "./MiDaS_small.pt")

device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
midas.to(device)
midas.eval()

midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")

if model_type == "DPT_Large" or model_type == "DPT_Hybrid":
    transform = midas_transforms.dpt_transform
else:
    transform = midas_transforms.small_transform

img = cv2.imread("dog.jpg")
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

# Timer begin
start = time.time()

input_batch = transform(img).to(device)

with torch.no_grad():
    prediction = midas(input_batch)
    prediction = torch.nn.functional.interpolate(
        prediction.unsqueeze(1),
        size=img.shape[:2],
        mode="bicubic",
        align_corners=False,
    ).squeeze()
    print(torch.min(prediction), torch.max(prediction))
    print(prediction.type())


output = prediction.cpu().numpy()
print(prediction.shape)
# Timer end
end = time.time()

print("Time Elapsed:", end-start)

f, axarr = plt.subplots(1,2)
axarr[0].imshow(img)
axarr[1].imshow(output)
plt.show()