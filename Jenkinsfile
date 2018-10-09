pipeline {
  agent any

  options {
    ansiColor('xterm')
  }

  stages {
    stage('SETUP') {
      steps {
        fileExists './uas.sh'
        sh './uas.sh nuke'
        sh './uas.sh run_env'
      }
    }
    stage('INITIAL CHECKS') {
      parallel {
        stage('ENVIRONMENT INFO') {
          steps {
            sh 'date'
            sh 'env'
            sh 'pwd'
          }
        }
        stage('LINT') {
          steps {
            sh './uas.sh lint --check'
          }
        }
      }
    }
    stage('BUILD') {
      parallel {
        stage('BUILD CONTROLS') {
          steps {
            sh './uas.sh build'
          }
        }
        stage('BUILD VISION') {
          steps {
            sh './uas.sh vision build'
          }
        }
        stage('BUILD GROUND') {
          steps {
            sh './uas.sh ground build'
          }
        }
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
            sh './uas.sh unittest'
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
