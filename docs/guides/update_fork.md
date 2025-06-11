# Updating a fork

For some of our ROS2-packages we use slightly modified Open-Source projects. In order to keep all of our Code in one place we fork these repositories to our own Gitlab. Since the repos are still maintained and updated on their main repo we have to update them from time to time.

To update a forked repository we first have to clone it locally.

```bash
git clone <gitlab-repo-url>
```

Now you need to have information about the original repository. Add the original source as a upstream repository.

```bash
git remote add upstream <original-repo-url>
```

Now you are ready to pull from the respective upstream branch again.

```bash
git pull upstream <branch>
```

Last but not least you need to push all merged changes to the Gitlab-Fork.

```bash
git push origin main
```