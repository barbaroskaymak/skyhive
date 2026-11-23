# ArduCopter SITL (software-in-the-loop) image used by SkyHive for swarm testing.
FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive
ARG ARDUPILOT_REF=Copter-4.5
RUN apt-get update && apt-get install -y --no-install-recommends \
      git ca-certificates python3 python3-pip python3-dev g++ make gawk pkg-config libtool \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir empy==3.3.4 pexpect future pyserial
RUN git clone --depth 1 --branch ${ARDUPILOT_REF} --recurse-submodules --shallow-submodules \
      https://github.com/ArduPilot/ardupilot.git /ardupilot
WORKDIR /ardupilot
RUN ./waf configure --board sitl && ./waf copter && \
    cp build/sitl/bin/arducopter /usr/local/bin/arducopter && \
    mkdir -p /opt/params && cp Tools/autotest/default_params/copter.parm /opt/params/copter.parm
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
# instance N serves MAVLink on TCP 5760 + 10*N
EXPOSE 5760 5770 5780 5790 5800 5810 5820 5830
ENTRYPOINT ["/entrypoint.sh"]
