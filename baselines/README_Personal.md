# RobustOptimizationSLAM

Steps to build the singularity file for cluster testing:

## 1- Update the docker image 

See the list of docker images
```
docker image ls
```

Run a container for editing the docker image
```
docker run -it [image] /bin/bash
```
Do the edits and when finished 

```
exit
```

See the list of containers running
```
docker ps -a
```

Commits the changes to new image
```
docker commit [container id] [new image name]
```

## 2- Push and convert to sif

```
docker push [slamemix/soa_bench_out:tagname]
```

Build sif image from docker file
```
sudo singularity build [name].sif docker://slamemix/soa_bench_out:v2
``` 

# Other useful command: 

Removes stopped container
``` 
docker container rm [container id]
``` 

Remove image from local
``` 
docker rmi [image]
``` 