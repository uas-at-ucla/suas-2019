pipeline {
  agent any
  stages {
    stage('Checkout') {
      steps {
        echo 'checkout'
      }
    }
    stage('Build') {
      steps {
        echo 'build'
      }
    }
    stage('Test SITL') {
      steps {
        echo 'test sitl'
      }
    }
    stage('Test HITL') {
      steps {
        echo 'test hitl'
      }
    }
    stage('Static Analyzer') {
      steps {
        echo 'static analyzer'
      }
    }
  }
}