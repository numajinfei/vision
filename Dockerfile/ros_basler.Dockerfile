FROM ros:galactic
LABEL maintainer="numajinfei@163.com"

# linux/amd64 or linux/arm64
ARG TARGETPLATFORM

RUN apt-get update && apt-get install -y --no-install-recommends \
  wget \
  && rm -rf /var/lib/apt/lists/*


# Install basler pylon6.2
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
    wget -O pylon.tar.gz https://www.baslerweb.com/fp-1615275588/media/a_b_testing_1/pylon_6.2.0.21487_x86_64_setup.tar.gz \
    && tar -xzvf pylon*.tar.gz \
    && mkdir /opt/pylon \
    && tar -C /opt/pylon -xzf ./pylon_*_x86_64.tar.gz \
    && rm -r pylon* \
    && echo "/opt/pylon/lib" >> /etc/ld.so.conf.d/Pylon.conf \
    && ldconfig; fi

RUN if [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
    wget -O pylon.tar.gz https://www.baslerweb.com/fp-1615276046/media/a_b_testing_1/pylon_6.2.0.21487_aarch64_setup.tar.gz \
    && tar -xzvf pylon*.tar.gz \
    && mkdir /opt/pylon \
    && tar -C /opt/pylon -xzf ./pylon_*_aarch64.tar.gz \
    && rm -r pylon* \
    && echo "/opt/pylon/lib" >> /etc/ld.so.conf.d/Pylon.conf \
    && ldconfig; fi
    


