# PX4 SITL running the SIH ("simulation-in-hardware") dynamics model.
#
# SIH computes the rigid-body physics inside PX4 itself, so there is no Gazebo,
# no rendering and no GPU: one process, ~7 MB RSS, boots in a couple of seconds.
# That is what makes this usable in CI, where the Gazebo images are ~3.3 GB.
#
# Build stage compiles PX4; the runtime stage keeps only the binary and the
# generated ROMFS, which is why the final image is ~120 MB instead of ~1.3 GB.

ARG PX4_VERSION=v1.14.3

FROM px4io/px4-dev-simulation-focal:2024-05-18 AS build
ARG PX4_VERSION
RUN git clone --depth 1 --branch ${PX4_VERSION} --recursive \
      https://github.com/PX4/PX4-Autopilot.git /src
WORKDIR /src
# `px4_sitl_default` builds the posix binary with simulator_sih linked in.
RUN make px4_sitl_default -j"$(nproc)"

FROM ubuntu:24.04
RUN apt-get update && apt-get install -y --no-install-recommends libstdc++6 \
 && rm -rf /var/lib/apt/lists/*

# `bin/` holds the px4 binary plus the px4-* multicall symlinks; rcS sources
# px4-alias.sh from PATH, so the whole directory has to come across, not just
# the binary. `etc/` is the *generated* ROMFS -- the source ROMFS lacks the
# airframe files the build produces.
COPY --from=build /src/build/px4_sitl_default/bin /px4/bin
COPY --from=build /src/build/px4_sitl_default/etc /px4/etc
ENV PATH="/px4/bin:${PATH}"

# PX4 writes parameters, dataman and logs into the working directory.
WORKDIR /rootfs

# 10040 = the sihsim_quadx airframe.
ENV PX4_SYS_AUTOSTART=10040 \
    PX4_SIMULATOR=sihsim \
    PX4_SIM_MODEL=sihsim_quadx

# Home position. NOTE: px4-rc.simulator does a raw `param set SIH_LOC_LAT0
# ${PX4_HOME_LAT}`, and that parameter is an int32 in 1e-7 degrees -- so these
# are scaled integers, NOT decimal degrees. Passing 47.397742 truncates to 47.
ENV PX4_HOME_LAT=473977420 \
    PX4_HOME_LON=85455940

# -d runs without the interactive pxh> console.
ENTRYPOINT ["/px4/bin/px4", "-d", "/px4/etc"]
