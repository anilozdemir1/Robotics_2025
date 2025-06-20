#!/usr/bin/env python3

'''
main.py
Doel: Detecteert ijsjes met behulp van YOLOv5 op een DepthAI OAK-D camera.
Toont coördinaten en oriëntatie van elk gedetecteerd ijsje.
'''

import os               # Werkt met bestands- en padcontrole (bijv. controleren of een bestand bestaat)
import json             # Om JSON-bestanden in te lezen (zoals modelconfiguraties en labels)
import cv2              # OpenCV voor beeldverwerking en visualisatie (bijv. tekenen van rechthoeken en tekst)
import depthai as dai   # DepthAI API om te communiceren met het OAK-D apparaat en AI pipeline op te zetten --> terminal: python3 -m pip install depthai --upgrade
import numpy as np      # Voor numerieke berekeningen zoals hoeken uit contouren berekenen
import time             # Voor tijdmetingen, o.a. berekening van FPS (frames per seconde)
import socket
import json

# === Functie om de rotatiehoek van het object binnen een ROI (Region of Interest) te bepalen ===
import cv2
import numpy as np

def calculate_angle_from_roi(roi):
    # Grijswaarden conversie en ruisreductie
    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray_roi, (5, 5), 0)

    # Adaptieve drempeling voor contourdetectie
    thresh_roi = cv2.adaptiveThreshold(
        blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
        cv2.THRESH_BINARY_INV, 15, 10
    )

    # Contouren vinden
    _, contours, _ = cv2.findContours(
        thresh_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    # Grootste contour selecteren
    largest_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest_contour) < 100:
        return None

    # Richtingsvector fitten aan de contour
    [vx, vy, _, _] = cv2.fitLine(
        largest_contour, cv2.DIST_L2, 0, 0.01, 0.01
    )

    # Roteer zodat 0 graden = verticaal omhoog, draai tegen de klok in
    angle_rad = np.arctan2(vx, vy)
    angle_deg = np.degrees(angle_rad)
    if angle_deg < 0:
        angle_deg += 360

# Haal hoek terug binnen 0-180 (zelfde lijn, geen richting)
    if angle_deg > 180:
        angle_deg -= 180


    return float(angle_deg)



# === Hoofdfunctie ===
def main():
    # === Model en configuratiebestanden ===
    # Dit zorgt dat het pad relatief is aan de locatie van dit script
    base_path = os.path.join(os.path.dirname(__file__), "..", "resources")
    blob_filename = os.path.join(base_path, "ultiemeijsjesdetectiesysteemv1iyolov5pytorch_openvino_2022.1_6shave.blob")  # YOLOv5 netwerk (gecompileerd voor OAK-D)
    json_filename = os.path.join(base_path, "ultiemeijsjesdetectiesysteemv1iyolov5pytorch.json") # Configuratiebestand met modelinstellingen en labels

    # === Bestanden controleren ===
    if not os.path.isfile(blob_filename):
        print(f'Error: BLOB file "{blob_filename}" niet gevonden.')
        return
    if not os.path.isfile(json_filename):
        print(f'Error: JSON file "{json_filename}" niet gevonden.')
        return

    # === Configuratie-instellingen inladen ===
    with open(json_filename) as f:
        config = json.load(f)

    # YOLOv5 specifieke metadata uit config halen
    nn_meta = config["nn_config"]["NN_specific_metadata"]
    numClasses = nn_meta["classes"]
    coordinateSize = nn_meta["coordinates"]
    anchors = nn_meta["anchors"]
    anchorMasks = nn_meta["anchor_masks"]
    iouThreshold = nn_meta["iou_threshold"]
    confidenceThreshold = nn_meta["confidence_threshold"]
    inputSizeX, inputSizeY = map(int, config["nn_config"]["input_size"].split("x"))

    # Label mapping om label-id om te zetten naar naam
    labelMap = config["mappings"]["labels"]
    if isinstance(labelMap, dict):
        labelMapFixed = {int(k): v for k, v in labelMap.items()}
    elif isinstance(labelMap, list):
        labelMapFixed = {i: v for i, v in enumerate(labelMap)}
    else:
        labelMapFixed = {}

    # === Pipeline aanmaken voor OAK-D ===
    pipeline = dai.Pipeline()

    # === Cameras en netwerk nodes aanmaken ===
    camRgb = pipeline.create(dai.node.ColorCamera)
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    detectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)

    # === Output streams definiëren ===
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutNN = pipeline.create(dai.node.XLinkOut)
    xoutDepth = pipeline.create(dai.node.XLinkOut)

    xoutRgb.setStreamName("rgb")
    xoutNN.setStreamName("detections")
    xoutDepth.setStreamName("depth")

    # === Camera instellingen ===
    camRgb.setPreviewSize(inputSizeX, inputSizeY)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

    # === YOLOv5 netwerk configureren ===
    detectionNetwork.setBlobPath(blob_filename)
    detectionNetwork.setConfidenceThreshold(confidenceThreshold)
    detectionNetwork.setBoundingBoxScaleFactor(0.5)
    detectionNetwork.setDepthLowerThreshold(100)
    detectionNetwork.setDepthUpperThreshold(5000)
    detectionNetwork.setNumClasses(numClasses)
    detectionNetwork.setCoordinateSize(coordinateSize)
    detectionNetwork.setAnchors(anchors)
    detectionNetwork.setAnchorMasks(anchorMasks)
    detectionNetwork.setIouThreshold(iouThreshold)

    # === Link nodes in de pipeline ===
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)
    camRgb.preview.link(detectionNetwork.input)
    stereo.depth.link(detectionNetwork.inputDepth)

    detectionNetwork.out.link(xoutNN.input)
    detectionNetwork.passthrough.link(xoutRgb.input)
    detectionNetwork.passthroughDepth.link(xoutDepth.input)

    # === Start het apparaat met de pipeline ===
    with dai.Device(pipeline) as device:
        qRgb = device.getOutputQueue("rgb", 4, False)
        qDet = device.getOutputQueue("detections", 4, False)
        qDepth = device.getOutputQueue("depth", 4, False)

        # === Voor fps-berekening ===
        startTime = time.monotonic()
        counter = 0
        fps = 0

        # Setup TCP-client verbinding naar vision_publisher (ROS Python 2)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('localhost', 9999))

        cv2.namedWindow("ultieme ijsjes detectiesysteem", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ultieme ijsjes detectiesysteem", 960, 540)  # voorbeeld 960x540 pixels

        while True:
            # === Frames ophalen ===
            inRgb = qRgb.get()
            inDet = qDet.get()
            inDepth = qDepth.get()

            frame = inRgb.getCvFrame()
            depthFrame = inDepth.getFrame()

            # Dieptebeeld visueel maken met kleurgradatie
            depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
            depthFrameColor = cv2.equalizeHist(depthFrameColor)
            depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

            # FPS bijhouden
            counter += 1
            if time.monotonic() - startTime >= 1:
                fps = counter / (time.monotonic() - startTime)
                counter = 0
                startTime = time.monotonic()

            # === Alle detecties verwerken ===
            for detection in inDet.detections:
                # Bounding box coördinaten
                x1 = int(detection.xmin * frame.shape[1])
                y1 = int(detection.ymin * frame.shape[0])
                x2 = int(detection.xmax * frame.shape[1])
                y2 = int(detection.ymax * frame.shape[0])
                label = labelMapFixed.get(detection.label, str(detection.label))

                # Rechthoek en label tekenen (ijsje benoemen)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} ({detection.confidence*100:.1f}%)", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # ROI extraheren voor hoekberekening
                x1_roi = max(0, x1)
                y1_roi = max(0, y1)
                x2_roi = min(frame.shape[1], x2)
                y2_roi = min(frame.shape[0], y2)
                roi = frame[y1_roi:y2_roi, x1_roi:x2_roi]
                angle = calculate_angle_from_roi(roi)

                # Gegevens naar ROS via socket
                try:
                    message = {
                        "label": label,
                        "x": detection.spatialCoordinates.x,
                        "y": detection.spatialCoordinates.y,
                        "z": detection.spatialCoordinates.z,
                        "rz": angle if angle is not None else 0.0
                    }
                    sock.sendall(json.dumps(message).encode('utf-8'))
                except Exception as e:
                    print("Fout bij verzenden naar ROS:", e)

                # Spatial (3D) coördinaten tonen
                cv2.putText(frame, f"X:{int(detection.spatialCoordinates.x)}mm", (x1, y2 + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame, f"Y:{int(detection.spatialCoordinates.y)}mm", (x1, y2 + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame, f"Z:{int(detection.spatialCoordinates.z)}mm", (x1, y2 + 45),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                # Hoek tonen als het beschikbaar is
                if angle is not None:
                    cv2.putText(frame, f"Angle: {angle:+.1f} deg", (x1, y2 + 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                            
            # FPS weergeven
            cv2.putText(frame, f"NN fps: {fps:.2f}", (5, frame.shape[0] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            # Frame weergeven
            cv2.imshow("ultieme ijsjes detectiesysteem", frame)
        
            if cv2.waitKey(1) == ord('q'):
                break

if __name__ == "__main__":
    main()