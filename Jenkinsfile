pipeline {
  agent any

  options {
    ansiColor('xterm')
  }

  stages {
    stage('SETUP') {
      steps {
        fileExists './uas'
        sh './uas nuke'
        sh './uas controls docker start'
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
            sh './uas lint --check'
          }
        }
      }
    }
    stage('BUILD') {
      parallel {
        stage('BUILD CONTROLS') {
          steps {
            sh './uas controls build'
          }
        }
        stage('BUILD VISION') {
          steps {
            sh './uas vision build'
          }
        }
        stage('BUILD GROUND') {
          steps {
            // Disable ground CI until we fix proto issues.
            //sh './uas ground build'
          }
        }
      }
    }
    stage('TEST') {
      parallel {
        stage('SITL TESTS') {
          steps {
            sh './uas controls sitl'
          }
        }
        stage('UNIT TESTS') {
          steps {
            echo 'unit tests'
            sh './uas unittest'
          }
        }
      }
    }
  }
  environment {
    PATH = "/usr/local/bin:/usr/bin:/bin:$PATH"
  }
}
