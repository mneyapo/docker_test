# Python Base Image from https://hub.docker.com/r/arm32v7/python/
FROM arm32v7/python:2.7.16-stretch

RUN which python
RUN which pip
#RUN pip install --upgrade pip 
#RUN pip install Adafruit_BBIO
RUN pip install smbus-cffi
# Install app dependencies

# Intall the rpi.gpio python module
RUN pip install --no-cache-dir rpi.gpio

RUN pip install spidev
RUN pip install --no-cache-dir pi-rc522
RUN pip list 
# Copy the Python Script 
COPY  python_script.py ./

# Trigger Python script
CMD ["python", "python_script.py"]
