import airsim
import time

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
#client.armDisarm(True)

client.simEnableWeather(True)

#client.simSetWeatherParameter(airsim.WeatherParameter.Fog, 1.0)
wind = airsim.Vector3r(20, 40, 0)
client.simSetWind(wind)

for i in range(1, 6):
    print("Remaining wind duration: %d seconds" % (6-i))
    time.sleep(1)

wind = airsim.Vector3r(0, 0, 0)
client.simSetWind(wind)
