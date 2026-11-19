FROM tek5/control
RUN pip install --break-system-packages --no-cache-dir torch==2.15.0 --index-url https://download.pytorch.org/whl/cpu
RUN pip install --break-system-packages --no-cache-dir gymnasium==1.4.0 stable-baselines3==2.9.2 numpy==1.26.4 matplotlib==3.9.2 contourpy==1.3.1 imageio-ffmpeg==0.8.1
ENV PYTHONPATH=/project/sim:/project/bonus
WORKDIR /project
