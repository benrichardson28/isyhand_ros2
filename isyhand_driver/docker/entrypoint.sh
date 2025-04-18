#!/bin/bash
set -e

# Set default UID/GID if not provided
HOST_UID=${HOST_USER_UID:-1000}
HOST_GID=${HOST_USER_GID:-1000}

# Create group if it doesn't exist
if ! getent group admin >/dev/null; then
    groupadd -g $HOST_GID admin
fi

# Create user if it doesn't exist
if ! id -u admin >/dev/null 2>&1; then
    useradd -u $HOST_UID -g $HOST_GID -s /bin/bash admin
fi

# Ensure home directory exists
if [ ! -d /home/admin ]; then
    mkdir -p /home/admin
    chown $HOST_UID:$HOST_GID /home/admin
fi

echo "admin ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/admin
chmod 0440 /etc/sudoers.d/admin

if [ ! -d /dev/ttyUSB0 ]; then
    sudo chmod 666 /dev/ttyUSB0
fi

# Ensure rosdep can access necessary directories
# Set proper permissions for rosdep's directories
mkdir -p /home/admin/.ros
mkdir -p /etc/ros/rosdep

# Make sure the admin user can read/write to those directories
chown -R admin:admin /home/admin/.ros
chown -R admin:admin /etc/ros/rosdep

# Run as the created user
service udev restart

cat << 'EOF' >> /home/admin/.bashrc
source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi
EOF

exec gosu admin "$@"
