FROM ros:galactic
LABEL maintainer="numajinfei@163.com"

# linux/amd64 or linux/arm64
ARG TARGETPLATFORM
#USER root
#COPY ./basler_setup.sh ./basler_setup.sh
#COPY ./pylon_6.2.0.21487_x86_64_setup.tar.gz ./pylon.tar.gz
# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
  wget \
  vim \
  && rm -rf /var/lib/apt/lists/*


# Install basler pylon6.2
RUN if [ "$TARGETPLATFORM" = "linux/amd64" ]; then \
    wget -O pylon.tar.gz https://www.baslerweb.com/fp-1615275588/media/a_b_testing_1/pylon_6.2.0.21487_x86_64_setup.tar.gz \
    && tar -xzvf pylon*.tar.gz \
    #tar -xzvf pylon*.tar.gz \
    && mkdir /opt/pylon \
    && tar -C /opt/pylon -xzf ./pylon_*_x86_64.tar.gz \
    && chmod 755 /opt/pylon \
    #&& sh -c '/bin/echo -e "yes" | /opt/pylon/share/pylon/setup-usb.sh' \
    #&& chmod +x ./basler_setup.sh \
    ##&& sh -c '/bin/echo -e "yes\nyes\n" | ./basler_setup.sh' \
    && rm -r pylon* ; fi
    #; fi
    
    
RUN  echo "platform is $TARGETPLATFORM, arch is $TARGETARCH, variant is $TARGETVARIANT, buildarch is $BUILDARCH, $(uname -p)" > /os.txt
#RUN if [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
RUN if [ "$(uname -p)" = "aarch64" ]; then \
    wget -O pylon.tar.gz https://www.baslerweb.com/fp-1615276046/media/a_b_testing_1/pylon_6.2.0.21487_aarch64_setup.tar.gz \
    && tar -xzvf pylon*.tar.gz \
    && mkdir /opt/pylon \
    && tar -C /opt/pylon -xzf ./pylon_*_aarch64.tar.gz \
    && chmod 755 /opt/pylon \
    #&& ./opt/pylon/share/pylon/setup-usb.sh \
    && rm -r pylon* ; fi

CMD ["sh", "-c", "echo \"/opt/pylon/bin/pylon-setup-env.sh\" >> ~/.bashrc && ldconfig"]

