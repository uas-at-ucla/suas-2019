#!/bin/bash

JENKINS_URL="https://uasatucla.org/jenkins/"
NAME=$1
USERID=jenkins_uasatucla
NODE_NAME=uasatucla.org
NODE_SLAVE_HOME="/home/$USERID/slave"
EXECUTORS=1
SSH_PORT=$2
CRED_ID="af7db496-c7b7-444e-ac32-1b4b5032d5bb"
AUTH_ID="comran:11c4ac3654e922b713aa40f735de841c14"
HOST_DOCKER_PATH=$3

echo "NAME IS $NAME"
echo "SSH_PORT IS $SSH_PORT"
echo "DOCKER PATH IS $HOST_DOCKER_PATH"

# Delete any existing nodes.
java \
  -jar /home/jenkins_uasatucla/jenkins-cli.jar \
  -s $JENKINS_URL \
  -auth $AUTH_ID \
  delete-node \
  $NAME

if [[ $? == 0 ]]
then
  echo "Deleted existing node with name $NAME"
fi

# Create another node for the current computer.
cat <<EOF | java -jar /home/jenkins_uasatucla/jenkins-cli.jar \
  -s $JENKINS_URL \
  -auth $AUTH_ID \
  create-node $NAME
<slave>
  <name>$NAME</name>
  <description></description>
  <remoteFS>${NODE_SLAVE_HOME}</remoteFS>
  <numExecutors>${EXECUTORS}</numExecutors>
  <mode>NORMAL</mode>
  <retentionStrategy class="hudson.slaves.RetentionStrategy$Always"/>
  <launcher class="hudson.plugins.sshslaves.SSHLauncher" plugin="ssh-slaves@1.5">
    <host>${NODE_NAME}</host>
    <port>${SSH_PORT}</port>
    <credentialsId>${CRED_ID}</credentialsId>
    <sshHostKeyVerificationStrategy class="hudson.plugins.sshslaves.verifiers.NonVerifyingKeyVerificationStrategy"/>
    <launchTimeoutSeconds>1200</launchTimeoutSeconds>
  </launcher>
  <label></label>
  <nodeProperties>
    <hudson.slaves.EnvironmentVariablesNodeProperty>
      <envVars serialization="custom">
        <unserializable-parents/>
        <tree-map>
          <default>
            <comparator class="hudson.util.CaseInsensitiveComparator"/>
          </default>
          <int>2</int>
          <string>HOST_ROOT_SEARCH</string>
          <string>/home/jenkins_uasatucla/slave</string>
          <string>HOST_ROOT_REPLACE</string>
          <string>$HOST_DOCKER_PATH/tools/cache/jenkins_slave</string>
        </tree-map>
      </envVars>
    </hudson.slaves.EnvironmentVariablesNodeProperty>
  </nodeProperties>
  <userId>${USERID}</userId>
</slave>
EOF

if [[ $? > 0 ]]
then
  exit 1
fi

# Template
# <?xml version='1.0' encoding='UTF-8'?>
# <slave>
#   <name>{{ item.name }}</name>
#   <description></description>
#   <remoteFS>/home/jenkins</remoteFS>
#   <numExecutors>4</numExecutors>
#   <mode>NORMAL</mode>
#   <retentionStrategy class="hudson.slaves.RetentionStrategy$Always"/>
#   <launcher class="hudson.plugins.sshslaves.SSHLauncher" plugin="ssh-slaves@1.9">
#     <host>{{ item.host }}</host>
#     <port>{{ item.port }}</port>
#     <credentialsId>new ssh key</credentialsId>
#     <maxNumRetries>0</maxNumRetries>
#     <retryWaitTime>0</retryWaitTime>
#     <sshHostKeyVerificationStrategy class="hudson.plugins.sshslaves.verifiers.NonVerifyingKeyVerificationStrategy"/>
#   </launcher>
#   <label>hetzner Ubuntu16.04</label>
#   <nodeProperties>
#     <hudson.slaves.EnvironmentVariablesNodeProperty>
#       <envVars serialization="custom">
#         <unserializable-parents/>
#         <tree-map>
#           <default>
#             <comparator class="hudson.util.CaseInsensitiveComparator"/>
#           </default>
#           <int>4</int>
#           <string>PYTHON26</string>
#           <string>/srv/python2.6/bin/python2.6</string>
#           <string>PYTHON27</string>
#           <string>/srv/python2.7/bin/python2.7</string>
#           <string>PYTHON36</string>
#           <string>/srv/python3.6/bin/python3.6</string>
#           <string>PATH</string>
#           <string>$PATH:/home/jenkins/.local/bin</string>
#         </tree-map>
#       </envVars>
#     </hudson.slaves.EnvironmentVariablesNodeProperty>
#   </nodeProperties>
#   <userId>gforcada</userId>
# </slave>
