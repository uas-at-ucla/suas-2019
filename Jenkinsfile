pipeline {
  agent any
  stages {
    stage('CHECKOUT') {
      steps {
        echo 'Checking if the run script exists.'
        fileExists './do.sh'
      }
    }
    stage('BUILD') {
      steps {
        echo 'Building the code.'
        sh './do.sh build'
      }
    }
    stage('SITL TEST') {
      steps {
        echo 'Test SITL'
      }
    }
    stage('HITL TEST') {
      steps {
        echo 'Test HITL'
      }
    }
    stage('STATIC ANALYZER') {
      steps {
        echo 'Static analyzer'
      }
    }
  }
  environment {
    PATH = "/usr/local/bin:/usr/bin:/bin:$PATH"
  }
}