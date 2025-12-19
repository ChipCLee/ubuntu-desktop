FROM ubuntu:latest

    # Install desktop environment and VNC server
    RUN apt-get update && apt-get install -y \
        ubuntu-desktop-minimal \
        tightvncserver
        # Add other desired packages

    # Configure VNC server (e.g., set password, start script)
    RUN mkdir -p ~/.vnc/ && echo "123" | vncpasswd -f > ~/.vnc/passwd && chmod 600 ~/.vnc/passwd
    COPY startup.sh /usr/local/bin/startup.sh
    RUN chmod +x /usr/local/bin/startup.sh

    # Expose VNC port
    EXPOSE 5901

    # Start VNC server when container runs
    CMD ["/usr/local/bin/startup.sh"]