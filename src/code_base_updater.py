from typing import List, Dict
from config import config
from common import ResourceConfig, Match
from match_finder import find_matches
from file_updater import update_file


def find_all_matches(config) -> List[Match]:
    all_matches: List[Match] = []
    for language in ['cpp', 'python']:
        for resource_type in ['topic', 'service']:
            for role in ['producer', 'consumer']:
                print(f'Grepping for {language} {resource_type} {role}')
                resource_config = ResourceConfig(
                    config, language, resource_type, role)
                resource_matches = find_matches(resource_config)
                print(f'Found {len(resource_matches)} matches found')
                all_matches += resource_matches
    return all_matches


def extract_files(all_matches: List[Match]) -> Dict[str, List[Match]]:
    files: Dict[str, List[Match]] = {}
    for match in all_matches:
        if match.file_path in files:
            files[match.file_path].append(match)
        else:
            files[match.file_path] = [match]
    return files


def update_codebase(config: ResourceConfig) -> None:
    print('Searching for all service/topic advertisers/subscribers in the codebase.')
    print('Depending on the codebase, this may take a while.')
    all_matches = find_all_matches(config)
    files = extract_files(all_matches)
    print(f'{len(files.keys())} files to update')
    for file in files:
        print(f'Updating {file}')
        update_file(files[file])


if __name__ == '__main__':
    update_codebase(config)
