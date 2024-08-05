# Levensthein distance is a well-known metric to quantify how different two strings are, e.g.
# used to check how different two languages are or simple spell-checkers
# https://en.wikipedia.org/wiki/Levenshtein_distance
class LevenshteinService:
    def calculate(self, s1: str, s2: str) -> int:
        """
        Calculate the Levenshtein distance between two strings.

        :param s1: First string
        :param s2: Second string
        :return: Levenshtein distance
        """
        if len(s1) < len(s2):
            return self.calculate(s2, s1)

        # Initialize distance matrix
        previous_row = range(len(s2) + 1)
        for i, c1 in enumerate(s1):
            current_row = [i + 1]
            for j, c2 in enumerate(s2):
                insertions = previous_row[j + 1] + 1
                deletions = current_row[j] + 1
                substitutions = previous_row[j] + (c1 != c2)
                current_row.append(min(insertions, deletions, substitutions))
            previous_row = current_row

        return previous_row[-1]
