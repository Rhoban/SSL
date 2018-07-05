pipeline {
    agent {
        dockerfile {
            additionalBuildArgs  '--build-arg SSH_KEY="`cat ~/.ssh/id_rsa`" --build-arg USER_ID=`id -u` --build-arg USER_GID=`id -g`'
        }
    }
    options { disableConcurrentBuilds() }

    environment {
        QN_COLOR_FAIL = '#7b0d1e'
        QN_COLOR_SUCCESS = '#44cf6c'
        QN_COLOR_NORMAL= '#439FE0'
    }

    stages {
        stage('prepare') {
            steps {
                sh '''
                    ./workspace setup
                    ./workspace install --https
                '''
            }
            post {
                failure {
                echo "Failed to prepare"
                }
            }
        }
        stage('build') {
            steps {
                sh '''
                    ./workspace build
                    ./workspace build_tests
                '''
           }
        }
        stage('tests') {
            steps {
                sh '''
                    ./workspace run_tests
                    ./workspace result_tests
                '''
            }

            post {
                always {
                    junit 'build_release/**/test_results/**/*.xml'
                }
            }
        }
    }
    post {
        always {
            deleteDir()
        }
    }
}
