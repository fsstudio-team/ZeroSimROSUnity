# Zero Sim distributable package

- [Zero Sim distributable package](#zero-sim-distributable-package)
  - [Publish new version](#publish-new-version)

## Publish new version
Github Actions is being used for creating new releases. Currently a workflow is triggered by pushing a tag that starts with `v`. So a tag `v0.1.6` would work. There's no automatic control on these tag versions, user needs to take care of it.\
The following regex is being used:\
`v*`

This workflow creates a new release draft in Github and adds a .tgz file associated to it. Maintainer needs to update the release notes accordingly and publish that release draft.

Note: In the future this will work automatically by using a more complex Github Actions workflow, which can autogenerate release notes, bump up versions and publish everything automatically.
