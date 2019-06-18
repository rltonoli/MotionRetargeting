# Base image
FROM python:3.7

# Updating repository sources
RUN apt-get update


# Copying requirements.txt file
COPY dev/gen-docker-img/environment.txt environment.txt
COPY . /home

# pip install
RUN pip install --no-cache -r environment.txt

# Exposing ports
EXPOSE 8889

# Running jupyter notebook
#CMD ["cd", "home"]
CMD ["jupyter", "notebook", "--no-browser", "--ip=0.0.0.0","--port=8889","--allow-root", "--NotebookApp.token=''"]
