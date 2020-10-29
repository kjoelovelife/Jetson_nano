sudo docker run --runtime nvidia -it --rm --network host \
--volume ~/nvdli-data:/nvdli-nano/data \
--volume /tmp/argus_socket:/tmp/argus_socket \
--device /dev/video0 \
nvcr.io/nvidia/dli/dli-nano-ai:v2.0.1-r32.4.4
