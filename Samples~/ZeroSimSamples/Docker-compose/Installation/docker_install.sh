# --------------------------------------------------- #
# --------------------------------------------------- #
# -------- INSTALL DOCKER ENGINE -------------------- #
# --------------------------------------------------- #
# --------------------------------------------------- #

# Older versions of Docker were called docker, docker.io, or docker-engine. 
# If these are installed, uninstall them:
sudo apt-get remove docker docker-engine docker.io containerd runc

# -------- SETUP THE REPOSITORY --------------------- #

# Update the apt package index and install packages to allow apt to use a repository over HTTPS:
sudo apt-get update
sudo apt-get install -y \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common

# Add Docker’s official GPG key:
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

# Verify that you now have the key with the fingerprint 9DC8 5822 9FC7 DD38 854A  E2D8 8D81 803C 0EBF CD88, 
# by searching for the last 8 characters of the fingerprint.
echo "Verify fingerprint is 9DC8 5822 9FC7 DD38 854A  E2D8 8D81 803C 0EBF CD88: "
sudo apt-key fingerprint 0EBFCD88

# Use the following command to set up the stable repository:
sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"

# -------- INSTALL DOCKER ENGINE -------------------- #

# Update the apt package index, and install the latest version of Docker Engine and containerd:
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io

# -------- OPTIONAL - VERIFY INSTALLATION ----------- #

# Download a test image and runs it in a container. 
# When the container runs, it prints an informational message and exits.
sudo docker run hello-world

# --------------------------------------------------- #
# --------------------------------------------------- #
# -------- INSTALL DOCKER COMPOSE ------------------- #
# --------------------------------------------------- #
# --------------------------------------------------- #

# Download the current stable release of Docker Compose:
sudo curl -L "https://github.com/docker/compose/releases/download/1.26.2/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose

# Apply executable permissions to the binary:
sudo chmod +x /usr/local/bin/docker-compose

# Note: If the command docker-compose fails after installation, check your path. 
# You can also create a symbolic link to /usr/bin or any other directory in your path.
# For example:
# sudo ln -s /usr/local/bin/docker-compose /usr/bin/docker-compose