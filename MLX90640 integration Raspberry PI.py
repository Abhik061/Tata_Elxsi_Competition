import cv2
import numpy as np
import board
import busio
import adafruit_mlx90640
 
# Setup MLX90640
i2c = busio.I2C(board.SCL, board.SDA)
mlx = adafruit_mlx90640.MLX90640(i2c)
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_2_HZ  # Adjust as needed
 
def get_thermal_image():
    frame = np.zeros((24*32,))
    mlx.getFrame(frame)
    data_array = np.reshape(frame, (24, 32))
    return data_array
 
def process_image(data_array):
    # Normalize data_array to 0-255 for image processing
    min_val, max_val = np.min(data_array), np.max(data_array)
    image = (data_array - min_val) / (max_val - min_val) * 255
    image = np.uint8(image)
    image = cv2.applyColorMap(image, cv2.COLORMAP_JET)
    # Resize the image to 640x560 to make more space for the GUI
    image = cv2.resize(image, (640, 560), interpolation=cv2.INTER_CUBIC)
    return image
 
def detect_hotspots(image, data_array, threshold=36):
    min_val, max_val = np.min(data_array), np.max(data_array)
    threshold_value = (threshold - min_val) / (max_val - min_val) * 255
    threshold_value = np.uint8(threshold_value)
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours
 
def main():
    while True:
        data_array = get_thermal_image()
        image = process_image(data_array)
        condition = "Normal"  # Default condition if no hotspots exceed the threshold
        max_overall_temp = np.max(data_array)  # Get the highest temperature for GUI display
 
        if max_overall_temp >= 80:#assumed upper limit to be 80
            condition = "Failure"  # Update condition to failure Detected
 
        hotspots = detect_hotspots(image, data_array, threshold=36)
        
        for cnt in hotspots:
            x, y, w, h = cv2.boundingRect(cnt)
            mask = np.zeros(data_array.shape, dtype=np.uint8)
            cv2.drawContours(mask, [cnt], -1, 255, thickness=cv2.FILLED)
            masked_temps = np.where(mask == 255, data_array, np.nan)
            max_temp = np.nanmax(masked_temps)
            if np.isnan(max_temp):
                continue
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            text_position = (x, max(0, y - 10))
            cv2.putText(image, f"{max_temp:.1f}C", text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
 
        # GUI Part: Display current max temperature and condition
        cv2.rectangle(image, (0, 520), (640, 560), (50, 50, 50), -1)
        cv2.putText(image, condition, (430, 540), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(image, f"Current Max Temp: {max_overall_temp:.1f}C", (10, 540), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
 
        cv2.imshow('Thermal Image', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
 
    cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main()
