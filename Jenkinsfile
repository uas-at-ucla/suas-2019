pipeline {
  agent any
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
    stage('TEST') {
      parallel {
        stage('SITL TESTS') {
          steps {
            echo 'Test SITL'
          }
        }
        stage('HITL TESTS') {
          steps {
            echo 'test hitl'
          }
        }
        stage('UNIT TESTS') {
          steps {
            echo 'unit tests'
          }
        }
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
  post {
    always {
      sh 'docker kill $(docker ps --filter status=running --format "{{.ID}}" --latest --filter name=uas_env) || true'

    }

  }
}