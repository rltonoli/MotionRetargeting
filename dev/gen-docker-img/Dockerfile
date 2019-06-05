# Base image
FROM python:3.7

# Updating repository sources
RUN apt-get update

# Copying requirements.txt file
COPY environment.txt environment.txt

# pip install 
RUN pip install --no-cache -r environment.txt

# Exposing ports
EXPOSE 8888

# Running jupyter notebook
CMD ["jupyter", "notebook", "--no-browser", "--ip=0.0.0.0", "--allow-root", "--NotebookApp.token=''"]