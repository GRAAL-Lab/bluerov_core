# If it fails remove build and install log
vcs import src < src/RAMI.repos
vcs pull src
# colcon build --symlink-install --continue-on-error
