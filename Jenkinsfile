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
    stage('FETCH') {
      steps {
        sh './tools/scripts/controls/exec.sh ./tools/scripts/controls/fetch_dependencies.sh'
      }
    }
    stage('STATIC ANALYSIS') {
      steps {
        withSonarQubeEnv('UAS@UCLA SonarQube') {
          sh "/home/jenkins_uasatucla/sonar-scanner-4.0.0.1744-linux/bin/sonar-scanner -Dsonar.projectKey=drone_code -Dsonar.host.url=https://uasatucla.org/sonarqube -Dsonar.sources=src,lib,tools/scripts "
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
            sh 'echo "Ground CI is disabled until we fix proto issues."'
          }
        }
      }
    }
    stage('TEST') {
      parallel {
        stage('SITL TESTS') {
          steps {
            sh 'echo fix this'
          }
        }
        stage('UNIT TESTS') {
          steps {
            sh 'echo fix this'
          }
        }
      }
    }
  }
  environment {
    PATH = "/usr/local/bin:/usr/bin:/bin:$PATH"
  }
}
