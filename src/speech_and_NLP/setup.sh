sudo apt update -y
sudo apt upgrade -y

sudo apt install cmake ffmpeg mecab libportaudio2 espeak -y

sudo apt-get install espeak

pip install -r requirements.txt

python -m spacy download en
