# Common bash utilities and setup.

# Quit immediately on any error.
set -e

SETUP=$(readlink -f $(dirname ${BASH_SOURCE[0]}))
REPO=$(readlink -f ${SETUP}/..)
RELEASE=$(lsb_release -a | sed -n -e 's/Codename:\t\s*\(\w\)/\1/p')

# $1 = log message
function log() {
    local C='\033[0;32m'
    local NC='\033[0m'
    printf "${C}$1${NC}\n"
}

# Save all output to a log file.
LOG_DIR=${SETUP}/logs
mkdir -p ${LOG_DIR}
exec &> >(tee ${LOG_DIR}/${LOG_NAME}-$(date +%F-%H-%M-%S).log)

# Log common info.
log "REPO: ${REPO}"
log "RELEASE: ${RELEASE}"
