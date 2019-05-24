# Git Workflow Info

* `master` - PR Protected - Only for cleaned up code from  `cleanup` branches
* `<competition-name>-cleanup` - Branched off of `release` for cleanup and testing before merging into `master`
* `release` - Competition branches are merged into here immediately after competitions
* `<competition-name>-competition` - Branched off of `develop` - code used during competition, quick fixes go here, at end of comp merged into `release`
* `develop` - Code used for drive practice, staging before the competition
* `<feature-name>` - For working on new features before merging into `develop`, should only be used by the author of the branch
