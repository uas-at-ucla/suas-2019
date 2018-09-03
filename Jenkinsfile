pipeline {
  agent any

  stages {
    stage('SETUP') {
      steps {
        sh 'date'
        sh 'env'
        sh 'pwd'
        fileExists './uas.sh'
        sh 'docker kill $(docker ps --filter status=running --format "{{.ID}}" --latest --filter name=uas_env) || true'
        sh './uas.sh run_env'
      }
    }
    stage('LINT') {
      steps {
        sh './uas.sh lint --check'
      }
    }
    stage('BUILD') {
      steps {
        sh './uas.sh build'
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
