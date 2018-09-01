pipeline {
  agent any
  stages {
    stage('Checkout') {
      steps {
        echo 'Checking if the run script exists.'
        fileExists './do.sh'
      }
    }
    stage('Build') {
      steps {
        echo 'Building the code.'
        sh 'PATH=${PATH}:/usr/local/bin:/usr/bin'
        catchError() {
          sh './do.sh build'
        }

      }
    }
    stage('Test SITL') {
      steps {
        echo 'Test SITL'
      }
    }
    stage('Test HITL') {
      steps {
        echo 'Test HITL'
      }
    }
    stage('Static Analyzer') {
      steps {
        echo 'Static analyzer'
      }
    }
  }
}