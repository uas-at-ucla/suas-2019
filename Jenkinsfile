pipeline {
  agent any
  environment {
    PATH = "/usr/local/bin:/usr/bin:/bin:$PATH"
  }
  stages {
    stage('SETUP') {
      steps {
        fileExists './do.sh'
        sh 'docker kill $(docker ps --filter status=running --format "{{.ID}}" --latest --filter name=uas_env) || true'
      }
    }
    stage('BUILD') {
      steps {
        sh './do.sh build'
      }
    }
    stage('TEST SITL') {
      steps {
        echo 'Test SITL'
      }
    }
    stage('TEST HITL') {
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
}
