FROM python:3.9.6-slim-bullseye

RUN mkdir -p /app

WORKDIR /app

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    git \
    openssh-client \
    libmariadb-dev \
    build-essential \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

ADD . .

# Install dependencies
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install -r requirements.txt

CMD ["python3", "main.py"]
